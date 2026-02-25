import sensor
import lcd
import gc
import time
from fpioa_manager import fm
from machine import UART

# ---------------- UART ----------------
fm.register(6, fm.fpioa.UART1_TX, force=True)
fm.register(8, fm.fpioa.UART1_RX, force=True)
uart_vision = UART(UART.UART1, 115200, 8, 0, 0, timeout=1000, read_buf_len=4096)

# ---------------- Image / Track Params ----------------
IMG_W = 320
IMG_H = 240
IMG_CX = IMG_W // 2

BLACK_THRESHOLD = (0, 8, -128, 127, -128, 127)
ROIS = (
    (0, 160, 320, 80, 0.55),
    (0, 110, 320, 50, 0.30),
    (0, 70, 320, 40, 0.15),
)

PIXELS_THRESHOLD = 30
AREA_THRESHOLD = 30
MAX_BLOB_PIXELS = 9000
EMA_ALPHA = 0.30

# K210-side turn friendly tracking (protocol unchanged: x + quality)
PREVIEW_GAIN = 0.55
ROI_PREDICT_GAINS = (0.15, 0.45, 0.75)
BLOB_CENTER_PENALTY = 2.0
EDGE_MARGIN_PX = 22
EDGE_QUALITY_MIN = 420
LOST_HOLD_MAX_FRAMES = 3
LOST_HOLD_QUALITY_START = 420
LOST_HOLD_QUALITY_DECAY = 90
PREDICT_VEL_ALPHA = 0.45
MAX_CX_STEP = 90

EXPOSURE_US = 50000
GAIN_DB = 14.0
BRIGHTNESS = 2
CONTRAST = 2
SATURATION = 1


def clamp(v, v_min, v_max):
    if v < v_min:
        return v_min
    if v > v_max:
        return v_max
    return v


def send_packet(err_norm, quality):
    x = int(clamp(err_norm * 1000.0, -1000.0, 1000.0))
    y = int(clamp(quality, 0, 1000))
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


def select_blob(blobs, expect_cx=None):
    if not blobs:
        return None
    valid = [b for b in blobs if b.pixels() <= MAX_BLOB_PIXELS]
    if not valid:
        return None
    if expect_cx is None:
        return max(valid, key=lambda b: b.pixels())

    best = None
    best_score = -999999
    for b in valid:
        score = b.pixels() - BLOB_CENTER_PENALTY * abs(b.cx() - expect_cx)
        if (best is None) or (score > best_score):
            best = b
            best_score = score
    return best


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
err_f = 0.0
last_sign = 1.0
frame_id = 0
last_cx_fused = float(IMG_CX)
last_cx_vel = 0.0
last_good_quality = 0
lost_hold_frames = 0
last_edge_dir = 0

while True:
    clock.tick()
    img = sensor.snapshot()
    img.draw_line(IMG_CX, 0, IMG_CX, IMG_H - 1, color=(0, 0, 255))

    hit_count = 0
    weighted_cx = 0.0
    weight_sum = 0.0
    pixel_sum = 0
    roi_centers = [None, None, None]
    roi_edge_hits = 0

    for idx, (x, y, w, h, wt) in enumerate(ROIS):
        img.draw_rectangle((x, y, w, h), color=(0, 255, 0))
        blobs = img.find_blobs(
            [BLACK_THRESHOLD],
            roi=(x, y, w, h),
            pixels_threshold=PIXELS_THRESHOLD,
            area_threshold=AREA_THRESHOLD,
            merge=True,
            margin=5,
        )

        expect_cx = clamp(last_cx_fused + last_cx_vel * ROI_PREDICT_GAINS[idx], 0, IMG_W - 1)
        b = select_blob(blobs, expect_cx)
        if not b:
            continue

        hit_count += 1
        pixel_sum += b.pixels()
        weighted_cx += b.cx() * wt
        weight_sum += wt
        roi_centers[idx] = b.cx()

        if (b.x() <= (x + EDGE_MARGIN_PX)) or ((b.x() + b.w()) >= (x + w - EDGE_MARGIN_PX)):
            roi_edge_hits += 1

        img.draw_rectangle(b.rect(), color=(255, 0, 0))
        img.draw_cross(b.cx(), b.cy(), color=(255, 255, 0))

    if hit_count > 0 and weight_sum > 0.0:
        cx_fused = weighted_cx / weight_sum
        cx_i = int(cx_fused)
        img.draw_line(cx_i, 0, cx_i, IMG_H - 1, color=(255, 0, 255))

        near_cx = roi_centers[0]
        if near_cx is None:
            near_cx = roi_centers[1] if (roi_centers[1] is not None) else roi_centers[2]
        far_cx = roi_centers[2]
        if far_cx is None:
            far_cx = roi_centers[1] if (roi_centers[1] is not None) else roi_centers[0]

        err = (cx_fused - IMG_CX) / float(IMG_CX)
        theta_term = 0.0
        if (near_cx is not None) and (far_cx is not None):
            theta_term = (far_cx - near_cx) / float(IMG_W)
        err_cmd = clamp(err + PREVIEW_GAIN * theta_term, -1.0, 1.0)

        alpha = EMA_ALPHA
        if hit_count >= 2:
            alpha = 0.35
        if roi_edge_hits > 0:
            alpha = 0.45
        err_f = (1.0 - alpha) * err_f + alpha * err_cmd
        last_sign = 1.0 if err_f >= 0.0 else -1.0

        cx_step = clamp(cx_fused - last_cx_fused, -MAX_CX_STEP, MAX_CX_STEP)
        last_cx_vel = (1.0 - PREDICT_VEL_ALPHA) * last_cx_vel + PREDICT_VEL_ALPHA * cx_step
        last_cx_fused = cx_fused

        edge_dir = 0
        if ((cx_fused < EDGE_MARGIN_PX * 2) or (roi_edge_hits > 0 and cx_fused < IMG_CX)):
            edge_dir = -1
        elif ((cx_fused > (IMG_W - EDGE_MARGIN_PX * 2)) or (roi_edge_hits > 0 and cx_fused > IMG_CX)):
            edge_dir = 1
        if edge_dir != 0:
            last_edge_dir = edge_dir

        hit_score = hit_count / 3.0
        area_score = clamp(pixel_sum / 2600.0, 0.0, 1.0)
        quality = int(620.0 * hit_score + 260.0 * area_score + 120.0 * clamp(weight_sum, 0.0, 1.0))
        if (roi_edge_hits > 0) and (hit_count >= 1):
            edge_boost = EDGE_QUALITY_MIN - (3 - hit_count) * 40
            if quality < edge_boost:
                quality = edge_boost
        quality = int(clamp(quality, 0, 1000))
        last_good_quality = quality
        lost_hold_frames = 0

        send_packet(err_f, quality)
        img.draw_string(6, 6, "TRACK E:%d Q:%d" % (int(err_f * 1000), quality), color=(255, 0, 0), scale=1)
        img.draw_string(6, 22, "CX:%d T:%d H:%d" % (cx_i, int(theta_term * 1000), hit_count), color=(255, 255, 0), scale=1)
    else:
        lost_hold_frames += 1
        if (lost_hold_frames <= LOST_HOLD_MAX_FRAMES) and (last_good_quality >= 220):
            pred_cx = last_cx_fused + last_cx_vel
            if last_edge_dir != 0:
                pred_cx += 18 * last_edge_dir
            pred_cx = clamp(pred_cx, 0, IMG_W - 1)
            pred_err = (pred_cx - IMG_CX) / float(IMG_CX)
            err_f = (1.0 - 0.55) * err_f + 0.55 * clamp(pred_err, -1.0, 1.0)
            q_hold = LOST_HOLD_QUALITY_START - (lost_hold_frames - 1) * LOST_HOLD_QUALITY_DECAY
            q_hold = int(clamp(q_hold, 0, 1000))
            send_packet(err_f, q_hold)
            img.draw_string(6, 6, "HOLD E:%d Q:%d" % (int(err_f * 1000), q_hold), color=(255, 128, 0), scale=1)
        else:
            err_f = 0.45 * last_sign
            send_packet(err_f, 0)
            img.draw_string(6, 6, "LOST E:%d Q:0" % int(err_f * 1000), color=(255, 0, 0), scale=1)

    img.draw_string(6, 38, "FPS:%.1f" % clock.fps(), color=(255, 255, 255), scale=1)
    lcd.display(img)

    frame_id += 1
    if (frame_id % 20) == 0:
        gc.collect()
