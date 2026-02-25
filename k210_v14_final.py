import sensor
import lcd
import gc
import time
from fpioa_manager import fm
from machine import UART

"""
K210 循迹脚本（配合 STM32 小车使用，串口协议与当前固件兼容）。

UART 数据包格式（兼容两种）：
  旧帧（7 字节） : [0xFF, 0xFE, x_hi, x_lo, q_hi, q_lo, checksum]
  扩展帧（11字节）: [0xFF, 0xFD, near_hi, near_lo, mid_hi, mid_lo, far_hi, far_lo, q_hi, q_lo, checksum]

字段说明：
  x / near / mid / far : 有符号 int16，归一化横向偏差，缩放后约为 [-1000, 1000]
  q                  : 无符号 int16，跟踪质量 [0, 1000]

当前版本包含：
  - 多 ROI 加权融合 + 一致性筛选
  - ROI 自适应阈值（抗环境光变化）
  - 扩展帧稳定输出（near/mid/far + quality，供 STM32 计算 theta）
  - 弯道贴内侧微偏置 + 丢线短时 HOLD 预测
"""

# ---------------- UART ----------------
fm.register(6, fm.fpioa.UART1_TX, force=True)
fm.register(8, fm.fpioa.UART1_RX, force=True)
uart_vision = UART(UART.UART1, 115200, 8, 0, 0, timeout=1000, read_buf_len=4096)
# 串口协议开关：True=扩展帧(near/mid/far/q)，False=旧帧(x/q)
# 当前 STM32 已支持扩展帧并使用 err+theta 双量控制。
# 这里默认开启扩展帧，同时在 K210 侧对 theta 做阻尼（低置信度时收缩 near/far 到 mid），
# 避免急弯/误检导致的突转。
VISION_EXT_PACKET_ENABLE = True

# ---------------- Image / Track Params ----------------
IMG_W = 320
IMG_H = 240
IMG_CX = IMG_W // 2

# 黑线识别备用 LAB 阈值（自适应阈值失败时回退使用）。
BLACK_THRESHOLD = (0, 8, -128, 127, -128, 127)

# ROI 格式：(x, y, w, h, weight)
# 近处 ROI 权重最大，因为它对当前转向影响最直接。
ROIS = (
    (0, 160, 320, 80, 0.55),
    (0, 110, 320, 50, 0.30),
    (0, 70, 320, 40, 0.15),
)

PIXELS_THRESHOLD = 30
AREA_THRESHOLD = 30
MAX_BLOB_PIXELS = 9000
# 发送给 STM32 的误差输出做 EMA 滤波（不是对原始 ROI 数据滤波）。
EMA_ALPHA = 0.30

# 抗环境光：每个 ROI 单独计算自适应 L 阈值（RGB565 -> LAB 的 L 通道）。
# L 上限由局部亮度统计得到，并做时间平滑，减少阈值抖动。
ADAPTIVE_L_THRESHOLD_ENABLE = True
ROI_L_MEAN_ALPHA = 0.22
ROI_L_HIGH_MIN = 6
ROI_L_HIGH_MAX = 40
ROI_DARK_OFFSET_BASE = 12
ROI_DARK_OFFSET_STD_GAIN = 0.9

# K210 侧循迹增强（主输出为 near/mid/far + quality，STM32 侧负责主控制）。
# 本地仍保留 err_f（用于调试显示 / LOST-HOLD 兜底），但不过度叠加复杂补偿。
PREVIEW_GAIN = 0.18
# ROI 预测选 blob，减少弯道时跳到错误 blob 的概率。
ROI_PREDICT_GAINS = (0.15, 0.45, 0.75)
BLOB_CENTER_PENALTY = 2.0
ROI_TARGET_PIXELS = (1500.0, 900.0, 650.0)
ROI_FILTER_ALPHA_BASE = 0.30
ROI_FILTER_ALPHA_CONF_GAIN = 0.42
ROI_FILTER_ALPHA_EDGE = 0.72
ROI_VIRTUAL_HOLD_FRAMES = 2
ROI_VIRTUAL_WEIGHT_SCALE = 0.30
# 预测窗口过滤：优先忽略离“预测位置”太远的 blob（常见于阴影误识别）。
# 注意这里不是按图像中心过滤；弯道时窗口会自动放宽，减少误杀真线。
PREDICT_GATE_BASE_PX = 42
PREDICT_GATE_MIN_PX = 28
PREDICT_GATE_MAX_PX = 130
PREDICT_GATE_VEL_GAIN = 0.70
PREDICT_GATE_OFFCENTER_GAIN = 0.35
PREDICT_GATE_ROI_EXTRA = (0, 16, 28)
# ROI 间一致性约束（用于抑制“某一块误检把整车带偏”）。
# 这些阈值是“软稳健”策略：优先剔除明显离谱的单块点，不追求几何拟合很严格。
ROI_CONSIST_NEAR_MID_BASE = 92
ROI_CONSIST_MID_FAR_BASE = 108
ROI_CONSIST_MID_INTERP_BASE = 58
ROI_CONSIST_EDGE_BONUS = 34
# 图像边缘质量保护（线贴边时不一定无效）。
EDGE_MARGIN_PX = 22
EDGE_QUALITY_MIN = 420
CURVE_QUALITY_MIN = 330
# 低置信度/点数不足时对 theta 做阻尼，避免扩展帧在弯道时突然带飞。
THETA_DAMP_MIN_SCALE = 0.28
THETA_DAMP_CONF_FULL = 0.70
# 弯道“贴内侧”轻微偏置：用于减轻能过弯但外轮压线的问题。
CURVE_TURNIN_TH = 0.055
CURVE_TURNIN_GAIN_PX = 220.0
CURVE_TURNIN_MAX_PX = 16.0
CURVE_TURNIN_EDGE_ATTEN = 0.65
# 短时预测 HOLD，用于弯道瞬时丢线过渡。
LOST_HOLD_MAX_FRAMES = 3
LOST_HOLD_QUALITY_START = 420
LOST_HOLD_QUALITY_DECAY = 90
PREDICT_VEL_ALPHA = 0.45
MAX_CX_STEP = 90
HOLD_EDGE_PUSH_PX = 18
# 本地 err_f 更新的限幅（主要防误检跳变；真正控制仍以 STM32 为主）。
ERR_SLEW_BASE = 0.10
ERR_SLEW_CURVE_GAIN = 0.34

# 相机参数（尽量固定/手动设置，提升复现性）。
EXPOSURE_US = 50000
GAIN_DB = 14.0
BRIGHTNESS = 2
CONTRAST = 2
SATURATION = 1


def clamp(v, v_min, v_max):
    """把标量限制到 [v_min, v_max] 区间内。"""
    if v < v_min:
        return v_min
    if v > v_max:
        return v_max
    return v


def cx_to_err_norm(cx):
    """像素 x 坐标转换为相对图像中心的归一化偏差。"""
    return clamp((float(cx) - float(IMG_CX)) / float(IMG_CX), -1.0, 1.0)


def build_roi_threshold(img, roi_rect, idx):
    """
    为单个 ROI 构建自适应阈值。

    先估计局部亮度，再计算“黑线 blob”的 L 上限阈值。
    同时对 ROI 亮度做低通滤波，避免每帧阈值来回跳。
    """
    if not ADAPTIVE_L_THRESHOLD_ENABLE:
        return BLACK_THRESHOLD
    try:
        stats = img.get_statistics(roi=roi_rect)
        try:
            l_mean = stats.l_mean()
            l_stdev = stats.l_stdev()
        except Exception:
            l_mean = stats.mean()
            l_stdev = stats.stdev()

        if roi_l_mean_f[idx] is None:
            roi_l_mean_f[idx] = float(l_mean)
        else:
            roi_l_mean_f[idx] = (1.0 - ROI_L_MEAN_ALPHA) * roi_l_mean_f[idx] + ROI_L_MEAN_ALPHA * float(l_mean)

        l_high = int(roi_l_mean_f[idx] - (ROI_DARK_OFFSET_BASE + ROI_DARK_OFFSET_STD_GAIN * float(l_stdev)))
        l_high = int(clamp(l_high, ROI_L_HIGH_MIN, ROI_L_HIGH_MAX))
        roi_l_thr_dbg[idx] = l_high
        return (0, l_high, -128, 127, -128, 127)
    except Exception:
        return BLACK_THRESHOLD


def send_packet(err_norm, quality, near_err=None, mid_err=None, far_err=None):
    """
    发送视觉数据到 STM32。

    - 旧帧：0xFF 0xFE + (x, q)
    - 扩展帧：0xFF 0xFD + (near, mid, far, q)
    """
    x = int(clamp(err_norm * 1000.0, -1000.0, 1000.0))
    y = int(clamp(quality, 0, 1000))

    if near_err is None:
        near_err = err_norm
    if mid_err is None:
        mid_err = err_norm
    if far_err is None:
        far_err = err_norm

    if not VISION_EXT_PACKET_ENABLE:
        x_u = x & 0xFFFF
        y_u = y & 0xFFFF
        pkt = bytearray(7)
        pkt[0] = 0xFF
        pkt[1] = 0xFE
        pkt[2] = (x_u >> 8) & 0xFF
        pkt[3] = x_u & 0xFF
        pkt[4] = (y_u >> 8) & 0xFF
        pkt[5] = y_u & 0xFF
        pkt[6] = (pkt[2] + pkt[3] + pkt[4] + pkt[5]) & 0xFF
        uart_vision.write(pkt)
        return

    vals = (
        int(clamp(near_err * 1000.0, -1000.0, 1000.0)),
        int(clamp(mid_err * 1000.0, -1000.0, 1000.0)),
        int(clamp(far_err * 1000.0, -1000.0, 1000.0)),
        y,
    )
    pkt = bytearray(11)
    pkt[0] = 0xFF
    pkt[1] = 0xFD
    csum = 0
    ofs = 2
    for v in vals:
        u = v & 0xFFFF
        hi = (u >> 8) & 0xFF
        lo = u & 0xFF
        pkt[ofs] = hi
        pkt[ofs + 1] = lo
        csum = (csum + hi + lo) & 0xFF
        ofs += 2
    pkt[10] = csum
    uart_vision.write(pkt)


def select_blob(blobs, expect_cx=None, gate_px=None):
    """
    在单个 ROI 中选择最可能的线 blob。

    - 无预测时：选面积最大的有效 blob
    - 有预测时：优先在预测窗口内筛选，再综合面积和与预测中心的距离评分
    """
    if not blobs:
        return None
    valid = [b for b in blobs if b.pixels() <= MAX_BLOB_PIXELS]
    if not valid:
        return None
    if expect_cx is None:
        return max(valid, key=lambda b: b.pixels())

    # 优先使用“靠近预测位置”的候选，过滤明显偏离的阴影/杂散黑块。
    # 若窗口内为空，则回退到全部 valid，避免在急弯时过度过滤导致直接丢线。
    if gate_px is not None:
        near_pred = [b for b in valid if abs(b.cx() - expect_cx) <= gate_px]
        if near_pred:
            valid = near_pred

    best = None
    best_score = -999999
    for b in valid:
        score = b.pixels() - BLOB_CENTER_PENALTY * abs(b.cx() - expect_cx)
        if (best is None) or (score > best_score):
            best = b
            best_score = score
    return best


def blend_available(a, b, wa):
    """双点融合；若其中一点缺失则自动回退到另一点。"""
    if a is None:
        return b
    if b is None:
        return a
    return a * wa + b * (1.0 - wa)


def blend_available_with_conf(a, a_conf, b, b_conf, wa):
    """融合数值和置信度（同样支持缺失回退）。"""
    if a is None and b is None:
        return None, 0.0
    if b is None:
        return a, a_conf
    if a is None:
        return b, b_conf
    return (a * wa + b * (1.0 - wa)), (a_conf * wa + b_conf * (1.0 - wa))


def slew_towards(cur, target, step):
    """限速跟随，抑制误检造成的瞬时跳变。"""
    d = target - cur
    if d > step:
        d = step
    elif d < -step:
        d = -step
    return cur + d


# ---------------- Sensor Init ----------------
lcd.init()
sensor.reset()
sensor.set_framesize(sensor.QVGA)
sensor.set_pixformat(sensor.RGB565)
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_auto_whitebal(False)

try:
    sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
except Exception:
    sensor.set_auto_exposure(True)

try:
    sensor.set_auto_gain(False, gain_db=GAIN_DB)
except Exception:
    sensor.set_auto_gain(True)

for fn, val in (
    ("set_brightness", BRIGHTNESS),
    ("set_contrast", CONTRAST),
    ("set_saturation", SATURATION),
):
    try:
        getattr(sensor, fn)(val)
    except Exception:
        pass

sensor.skip_frames(time=1500)


# ---------------- Main Loop ----------------
clock = time.clock()

# 运行态状态量（跨帧保留）。
err_f = 0.0
last_sign = 1.0
frame_id = 0
last_cx_fused = float(IMG_CX)
last_cx_vel = 0.0
last_curve_px = 0.0
last_good_quality = 0
lost_hold_frames = 0
last_edge_dir = 0
roi_l_mean_f = [None] * len(ROIS)
roi_l_thr_dbg = [BLACK_THRESHOLD[1]] * len(ROIS)
roi_cx_filt = [float(IMG_CX)] * len(ROIS)
roi_miss_age = [99] * len(ROIS)
last_near_pkt_cx = float(IMG_CX)
last_mid_pkt_cx = float(IMG_CX)
last_far_pkt_cx = float(IMG_CX)

while True:
    clock.tick()
    img = sensor.snapshot()

    # 调试：图像中心线（相机坐标中心）。
    img.draw_line(IMG_CX, 0, IMG_CX, IMG_H - 1, color=(0, 0, 255))

    # 当前帧 ROI 原始检测结果（先存储，后做一致性筛选再融合）。
    roi_centers = [None] * len(ROIS)      # 已做轻度滤波后的 cx（本帧检测到才有值）
    roi_expects = [None] * len(ROIS)
    roi_pixels = [0] * len(ROIS)
    roi_edge_flags = [0] * len(ROIS)
    roi_conf = [0.0] * len(ROIS)
    roi_real_flags = [0] * len(ROIS)

    # 1) 扫描所有 ROI，并在每个 ROI 中选一个 blob（优先靠近预测中心）。
    for idx, (x, y, w, h, wt) in enumerate(ROIS):
        img.draw_rectangle((x, y, w, h), color=(0, 255, 0))
        roi_thr = build_roi_threshold(img, (x, y, w, h), idx)
        blobs = img.find_blobs(
            [roi_thr],
            roi=(x, y, w, h),
            pixels_threshold=PIXELS_THRESHOLD,
            area_threshold=AREA_THRESHOLD,
            merge=True,
            margin=5,
        )

        expect_cx = clamp(last_cx_fused + last_cx_vel * ROI_PREDICT_GAINS[idx], 0, IMG_W - 1)
        roi_expects[idx] = expect_cx
        # 动态预测窗口：直线时更严格，转弯/高速横移/偏离中心时自动放宽。
        gate_px = PREDICT_GATE_BASE_PX
        gate_px += PREDICT_GATE_VEL_GAIN * abs(last_cx_vel)
        gate_px += PREDICT_GATE_OFFCENTER_GAIN * abs(expect_cx - IMG_CX)
        gate_px += PREDICT_GATE_ROI_EXTRA[idx]
        gate_px = int(clamp(gate_px, PREDICT_GATE_MIN_PX, PREDICT_GATE_MAX_PX))
        b = select_blob(blobs, expect_cx, gate_px)
        if not b:
            roi_miss_age[idx] += 1
            continue

        raw_cx = float(b.cx())
        roi_pixels[idx] = b.pixels()
        roi_real_flags[idx] = 1

        if (b.x() <= (x + EDGE_MARGIN_PX)) or ((b.x() + b.w()) >= (x + w - EDGE_MARGIN_PX)):
            roi_edge_flags[idx] = 1

        # ROI 点置信度：预测一致性 + 面积（线宽/局部遮挡会影响面积，因此只作软约束）。
        pred_score = 1.0 - abs(raw_cx - expect_cx) / float(max(gate_px, 1))
        pred_score = clamp(pred_score, 0.0, 1.0)
        area_score = clamp(float(b.pixels()) / ROI_TARGET_PIXELS[idx], 0.0, 1.0)
        conf = 0.18 + 0.47 * pred_score + 0.35 * area_score
        if roi_edge_flags[idx]:
            conf += 0.08
        # 超宽黑块常见于阴影/拼接边，做轻惩罚（不直接杀掉，避免急弯误伤）。
        if (b.w() > (w * 0.72)) and (b.pixels() > (ROI_TARGET_PIXELS[idx] * 0.9)):
            conf -= 0.12
        conf = clamp(conf, 0.0, 1.0)
        roi_conf[idx] = conf

        # 对各 ROI cx 做轻度时域滤波，降低抖动；边缘命中时提高响应，避免弯道修正滞后。
        alpha_roi = ROI_FILTER_ALPHA_BASE + ROI_FILTER_ALPHA_CONF_GAIN * conf
        if roi_edge_flags[idx] and (alpha_roi < ROI_FILTER_ALPHA_EDGE):
            alpha_roi = ROI_FILTER_ALPHA_EDGE
        if roi_miss_age[idx] >= 3:
            roi_cx_filt[idx] = raw_cx
        else:
            roi_cx_filt[idx] = (1.0 - alpha_roi) * roi_cx_filt[idx] + alpha_roi * raw_cx
        roi_centers[idx] = roi_cx_filt[idx]
        roi_miss_age[idx] = 0

        img.draw_rectangle(b.rect(), color=(255, 0, 0))
        img.draw_cross(b.cx(), b.cy(), color=(255, 255, 0))

    # 1.5) ROI 一致性筛选：去掉明显离谱的单块误检（常见于阴影）。
    roi_used = [c is not None for c in roi_centers]
    if any(roi_used):
        edge_bonus = ROI_CONSIST_EDGE_BONUS if any((roi_used[i] and roi_edge_flags[i]) for i in (0, 1, 2)) else 0

        def pred_err(idx):
            if (roi_centers[idx] is None) or (roi_expects[idx] is None):
                return 9999.0
            return abs(float(roi_centers[idx]) - float(roi_expects[idx]))

        # near-mid 一致性：通常优先相信 near；若 near 明显比 mid 更偏离预测，也允许丢 near。
        if roi_used[0] and roi_used[1]:
            th_nm = ROI_CONSIST_NEAR_MID_BASE + edge_bonus + 0.22 * abs(roi_centers[0] - IMG_CX)
            if abs(roi_centers[1] - roi_centers[0]) > th_nm:
                if pred_err(1) + 10.0 < pred_err(0):
                    roi_used[0] = False
                else:
                    roi_used[1] = False

        # mid-far 一致性
        if roi_used[1] and roi_used[2]:
            th_mf = ROI_CONSIST_MID_FAR_BASE + edge_bonus + 0.18 * abs(roi_centers[1] - IMG_CX)
            if abs(roi_centers[2] - roi_centers[1]) > th_mf:
                if pred_err(2) + 10.0 < pred_err(1):
                    roi_used[1] = False
                else:
                    roi_used[2] = False

        # near-far 粗一致性 + 中点插值检查（主要防单块阴影把 theta 拉飞）
        if roi_used[0] and roi_used[2]:
            th_nf = (ROI_CONSIST_NEAR_MID_BASE + ROI_CONSIST_MID_FAR_BASE) + edge_bonus
            if abs(roi_centers[2] - roi_centers[0]) > th_nf:
                if pred_err(2) + 10.0 < pred_err(0):
                    roi_used[0] = False
                else:
                    roi_used[2] = False
        if roi_used[0] and roi_used[1] and roi_used[2]:
            mid_pred = 0.5 * (roi_centers[0] + roi_centers[2])
            th_interp = ROI_CONSIST_MID_INTERP_BASE + 0.5 * edge_bonus
            if abs(roi_centers[1] - mid_pred) > th_interp:
                # 丢掉预测偏差更大的那个（通常是阴影误检块）
                if pred_err(1) > pred_err(2) + 6.0:
                    roi_used[1] = False
                elif pred_err(2) > pred_err(1) + 6.0:
                    roi_used[2] = False
                else:
                    roi_used[1] = False

    # 被一致性筛掉的点也按“本帧缺失”处理，便于后续虚拟点接管。
    for idx in range(len(ROIS)):
        if roi_real_flags[idx] and (not roi_used[idx]):
            roi_miss_age[idx] += 1

    # 1.6) 为输出构建“轨迹点”（真实点优先；短时缺失时可用历史预测虚拟点补洞）。
    roi_track_centers = [None] * len(ROIS)
    roi_track_conf = [0.0] * len(ROIS)
    roi_virtual_flags = [0] * len(ROIS)
    for idx in range(len(ROIS)):
        if roi_used[idx]:
            roi_track_centers[idx] = roi_centers[idx]
            roi_track_conf[idx] = roi_conf[idx]
            continue
        if (roi_miss_age[idx] <= ROI_VIRTUAL_HOLD_FRAMES) and (last_good_quality >= 220):
            pred_cx = roi_cx_filt[idx] + last_cx_vel * ROI_PREDICT_GAINS[idx]
            # 使用上一帧曲率做很小的方向延续，帮助撑过“弯道瞬时出画”。
            if idx == 0:
                pred_cx -= 0.15 * last_curve_px
            elif idx == 2:
                pred_cx += 0.25 * last_curve_px
            pred_cx = clamp(pred_cx, 0, IMG_W - 1)
            roi_track_centers[idx] = pred_cx
            roi_track_conf[idx] = clamp(0.34 - 0.10 * (roi_miss_age[idx] - 1), 0.10, 0.34)
            roi_virtual_flags[idx] = 1

    # 1.7) 用轨迹点做融合（近 ROI 主导，中/远 ROI 可贡献但不允许单点带飞）。
    hit_count = 0        # 真实点数量（用于质量和状态判断）
    track_count = 0      # 真实+虚拟点数量（用于计算 near/mid/far）
    weighted_cx = 0.0
    weight_sum = 0.0
    pixel_sum = 0
    roi_edge_hits = 0
    conf_sum_real = 0.0
    near_anchor = roi_track_centers[0]
    for idx, (x, y, w, h, wt) in enumerate(ROIS):
        cx_use = roi_track_centers[idx]
        if cx_use is None:
            continue

        wt_eff = wt
        if (near_anchor is not None) and (idx != 0):
            # 近 ROI 主导：中/远 ROI 偏离 near 越大，权重越低（但不直接清零）。
            near_gate = ROI_CONSIST_NEAR_MID_BASE + (ROI_CONSIST_EDGE_BONUS if roi_edge_flags[0] else 0)
            near_gate += 0.20 * abs(near_anchor - IMG_CX)
            diff_ratio = abs(cx_use - near_anchor) / float(max(near_gate, 1.0))
            diff_ratio = clamp(diff_ratio, 0.0, 1.0)
            wt_eff *= (1.0 - 0.45 * diff_ratio)
        wt_eff *= (0.45 + 0.55 * roi_track_conf[idx])
        if roi_virtual_flags[idx]:
            wt_eff *= ROI_VIRTUAL_WEIGHT_SCALE

        track_count += 1
        weighted_cx += cx_use * wt_eff
        weight_sum += wt_eff
        if roi_used[idx]:
            hit_count += 1
            pixel_sum += roi_pixels[idx]
            conf_sum_real += roi_conf[idx]
            if roi_edge_flags[idx]:
                roi_edge_hits += 1

    # 2) TRACK：至少有一个 ROI 看见线 -> 融合位置并输出 x + quality。
    if hit_count > 0 and weight_sum > 0.0:
        cx_fused = weighted_cx / weight_sum
        cx_i = int(cx_fused)
        img.draw_line(cx_i, 0, cx_i, IMG_H - 1, color=(255, 0, 255))

        # 构造 near/mid/far 点（优先真实点；缺失时允许由虚拟点/邻近点回填）。
        near_cx, near_conf = blend_available_with_conf(
            roi_track_centers[0], roi_track_conf[0],
            roi_track_centers[1], roi_track_conf[1],
            0.75
        )
        if roi_track_centers[1] is not None:
            mid_cx = roi_track_centers[1]
            mid_conf = roi_track_conf[1]
        else:
            mid_cx, mid_conf = blend_available_with_conf(
                roi_track_centers[0], roi_track_conf[0],
                roi_track_centers[2], roi_track_conf[2],
                0.55
            )
        far_cx, far_conf = blend_available_with_conf(
            roi_track_centers[2], roi_track_conf[2],
            roi_track_centers[1], roi_track_conf[1],
            0.75
        )
        if mid_cx is None:
            mid_cx = cx_fused
            mid_conf = 0.18
        if near_cx is None:
            near_cx = mid_cx
            near_conf = 0.15
        if far_cx is None:
            far_cx = mid_cx
            far_conf = 0.15

        near_cx = float(clamp(near_cx, 0, IMG_W - 1))
        mid_cx = float(clamp(mid_cx, 0, IMG_W - 1))
        far_cx = float(clamp(far_cx, 0, IMG_W - 1))
        near_conf = clamp(near_conf, 0.0, 1.0)
        mid_conf = clamp(mid_conf, 0.0, 1.0)
        far_conf = clamp(far_conf, 0.0, 1.0)

        # 先基于 raw near/far 得到曲率；强弯时给 near/mid 一个“贴内侧”轻微偏置，
        # 目的是减轻“能过弯但外轮压线”的情况（转向来得太晚/太外）。
        theta_term_raw = (far_cx - near_cx) / float(IMG_W)
        turnin_bias_px = 0.0
        if (hit_count >= 2) and (min(near_conf, mid_conf) >= 0.30):
            theta_abs = abs(theta_term_raw)
            if theta_abs > CURVE_TURNIN_TH:
                turnin_bias_px = (theta_abs - CURVE_TURNIN_TH) * CURVE_TURNIN_GAIN_PX
                turnin_bias_px = clamp(turnin_bias_px, 0.0, CURVE_TURNIN_MAX_PX)
                if roi_edge_hits > 0:
                    turnin_bias_px *= CURVE_TURNIN_EDGE_ATTEN
                turnin_sign = 1.0 if theta_term_raw >= 0.0 else -1.0
                near_cx = clamp(near_cx + turnin_sign * turnin_bias_px, 0, IMG_W - 1)
                mid_cx = clamp(mid_cx + turnin_sign * (0.55 * turnin_bias_px), 0, IMG_W - 1)

        # 扩展帧稳定关键：当 far/near 置信度不足时，把 near/far 收缩到 mid，主动减小 theta。
        theta_conf = min(near_conf, far_conf)
        if hit_count < 2:
            theta_scale = 0.0
        elif theta_conf >= THETA_DAMP_CONF_FULL:
            theta_scale = 1.0
        else:
            theta_scale = THETA_DAMP_MIN_SCALE + (1.0 - THETA_DAMP_MIN_SCALE) * (theta_conf / THETA_DAMP_CONF_FULL)
            theta_scale = clamp(theta_scale, THETA_DAMP_MIN_SCALE, 1.0)
        near_cx = mid_cx + (near_cx - mid_cx) * theta_scale
        far_cx = mid_cx + (far_cx - mid_cx) * theta_scale

        # 本地 err_f 仅用于调试/HOLD；实际控制以 STM32 侧 err+theta 为主。
        theta_term = (far_cx - near_cx) / float(IMG_W)
        err = ((0.65 * near_cx + 0.35 * mid_cx) - IMG_CX) / float(IMG_CX)
        turn_term = PREVIEW_GAIN * theta_term
        err_cmd = clamp(err + turn_term, -1.0, 1.0)

        alpha = EMA_ALPHA
        if hit_count >= 2:
            alpha = 0.35
        if (roi_edge_hits > 0) or (abs(theta_term) > 0.085):
            alpha = 0.45
        err_lpf = (1.0 - alpha) * err_f + alpha * err_cmd
        err_slew = ERR_SLEW_BASE + ERR_SLEW_CURVE_GAIN * abs(theta_term)
        if roi_edge_hits > 0:
            err_slew += 0.05
        err_slew = clamp(err_slew, 0.07, 0.25)
        err_f = slew_towards(err_f, err_lpf, err_slew)
        turn_hint = err_f + 0.7 * theta_term
        last_sign = 1.0 if turn_hint >= 0.0 else -1.0

        cx_step = clamp(cx_fused - last_cx_fused, -MAX_CX_STEP, MAX_CX_STEP)
        last_cx_vel = (1.0 - PREDICT_VEL_ALPHA) * last_cx_vel + PREDICT_VEL_ALPHA * cx_step
        last_cx_fused = cx_fused
        last_curve_px = (1.0 - 0.35) * last_curve_px + 0.35 * (far_cx - near_cx)

        # 记录最近一次有效边缘方向，供 HOLD 预测时使用。
        edge_dir = 0
        if ((cx_fused < EDGE_MARGIN_PX * 2) or (roi_edge_hits > 0 and cx_fused < IMG_CX)):
            edge_dir = -1
        elif ((cx_fused > (IMG_W - EDGE_MARGIN_PX * 2)) or (roi_edge_hits > 0 and cx_fused > IMG_CX)):
            edge_dir = 1
        if edge_dir != 0:
            last_edge_dir = edge_dir

        # 质量分数组成：ROI 命中数 + blob 面积 + 加权覆盖度。
        # STM32 会用它决定 TRACK / SEARCH / LOST。
        hit_score = hit_count / 3.0
        area_score = clamp(pixel_sum / 2600.0, 0.0, 1.0)
        conf_score = (conf_sum_real / float(hit_count)) if (hit_count > 0) else 0.0
        quality = int(500.0 * hit_score + 220.0 * area_score + 140.0 * clamp(weight_sum, 0.0, 1.0) + 140.0 * conf_score)
        if (hit_count >= 2) and (abs(theta_term) > CURVE_TURNIN_TH):
            curve_floor = CURVE_QUALITY_MIN + (20 if theta_scale >= 0.7 else 0)
            if quality < curve_floor:
                quality = curve_floor
        if (roi_edge_hits > 0) and (hit_count >= 1):
            # 线贴边但仍有效时，避免质量过早塌掉。
            edge_boost = EDGE_QUALITY_MIN - (3 - hit_count) * 40
            if quality < edge_boost:
                quality = edge_boost
        quality = int(clamp(quality, 0, 1000))
        last_good_quality = quality
        lost_hold_frames = 0

        last_near_pkt_cx = near_cx
        last_mid_pkt_cx = mid_cx
        last_far_pkt_cx = far_cx
        near_err_pkt = cx_to_err_norm(near_cx)
        mid_err_pkt = cx_to_err_norm(mid_cx)
        far_err_pkt = cx_to_err_norm(far_cx)
        send_packet(err_f, quality, near_err_pkt, mid_err_pkt, far_err_pkt)
        img.draw_string(6, 6, "TRACK E:%d Q:%d" % (int(err_f * 1000), quality), color=(255, 0, 0), scale=1)
        img.draw_string(
            6, 22,
            "CX:%d T:%d B:%d H:%d" % (cx_i, int(theta_term * 1000), int(turnin_bias_px), hit_count),
            color=(255, 255, 0), scale=1
        )
    else:
        # 3) LOST/HOLD：本帧未检测到线。
        # 先尝试短时预测 HOLD，再退化为 Q=0（完全丢线）。
        lost_hold_frames += 1
        if (lost_hold_frames <= LOST_HOLD_MAX_FRAMES) and (last_good_quality >= 220):
            pred_mid_cx = last_mid_pkt_cx + last_cx_vel
            if last_edge_dir != 0:
                # 把预测往上次边缘方向推一点，更容易撑过弯道出画瞬间。
                pred_mid_cx += HOLD_EDGE_PUSH_PX * last_edge_dir
            pred_curve_px = last_curve_px * 0.88
            pred_near_cx = pred_mid_cx - 0.50 * pred_curve_px
            pred_far_cx = pred_mid_cx + 0.50 * pred_curve_px
            pred_near_cx = clamp(pred_near_cx, 0, IMG_W - 1)
            pred_mid_cx = clamp(pred_mid_cx, 0, IMG_W - 1)
            pred_far_cx = clamp(pred_far_cx, 0, IMG_W - 1)
            pred_err = ((0.65 * pred_near_cx + 0.35 * pred_mid_cx) - IMG_CX) / float(IMG_CX)
            err_hold = (1.0 - 0.55) * err_f + 0.55 * clamp(pred_err, -1.0, 1.0)
            err_f = slew_towards(err_f, err_hold, 0.20)
            q_hold = LOST_HOLD_QUALITY_START - (lost_hold_frames - 1) * LOST_HOLD_QUALITY_DECAY
            q_hold = int(clamp(q_hold, 0, 1000))
            send_packet(
                err_f, q_hold,
                cx_to_err_norm(pred_near_cx),
                cx_to_err_norm(pred_mid_cx),
                cx_to_err_norm(pred_far_cx)
            )
            img.draw_string(6, 6, "HOLD E:%d Q:%d" % (int(err_f * 1000), q_hold), color=(255, 128, 0), scale=1)
        else:
            # 保持误差符号连续，帮助 STM32 按上一次转向方向搜索。
            err_f = 0.45 * last_sign
            send_packet(err_f, 0, err_f, err_f, err_f)
            img.draw_string(6, 6, "LOST E:%d Q:0" % int(err_f * 1000), color=(255, 0, 0), scale=1)

    # 调试叠加信息：
    #   FPS     : 相机主循环帧率
    #   L:a/b/c : 3 个 ROI 的自适应 L 阈值
    img.draw_string(6, 54, "L:%d/%d/%d" % (roi_l_thr_dbg[0], roi_l_thr_dbg[1], roi_l_thr_dbg[2]), color=(0, 255, 255), scale=1)
    img.draw_string(6, 38, "FPS:%.1f" % clock.fps(), color=(255, 255, 255), scale=1)
    lcd.display(img)

    frame_id += 1
    if (frame_id % 20) == 0:
        # 周期性 GC，减少长时间运行后的堆碎片导致的卡顿。
        gc.collect()
