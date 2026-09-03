# 第三方声明与许可边界

本文件是仓库内第三方代码许可线索的保守、非穷尽清单，不构成法律意见，也不替代各源文件中的版权或许可声明。未列出某个文件不表示它没有第三方权利或许可义务。请保留现有声明；在完成来源和授权核实前，不应推断、补写或重新许可相关代码。

## 仓库中已有许可文件

- STM32F1 HAL 驱动：[`Drivers/STM32F1xx_HAL_Driver/LICENSE.txt`](Drivers/STM32F1xx_HAL_Driver/LICENSE.txt)
- CMSIS：[`Drivers/CMSIS/LICENSE.txt`](Drivers/CMSIS/LICENSE.txt)
- STM32F1 CMSIS 设备文件：[`Drivers/CMSIS/Device/ST/STM32F1xx/LICENSE.txt`](Drivers/CMSIS/Device/ST/STM32F1xx/LICENSE.txt)
- FreeRTOS：[`Middlewares/Third_Party/FreeRTOS/Source/LICENSE`](Middlewares/Third_Party/FreeRTOS/Source/LICENSE)

这些许可文件各自只应按其文本和适用范围理解；本清单不声称其中任何一个许可覆盖整个仓库。

## 文件内嵌许可

- cJSON：`Core/Inc/cJSON.h` 与 `Core/Src/cJSON.c` 在各自文件头内嵌了 MIT 许可文本及 Dave Gamble 和 cJSON contributors 的版权声明。使用或分发时应保留文件中的许可与版权文本。

## 许可或来源待核实

以下文件包含来源、版权或许可线索，但仓库中未见足以据此确认再分发条件的对应许可材料。本文件不猜测其许可：

- STM32CubeMX 生成的 `Core/` 下部分文件、`startup_stm32f103xb.s` 等文件：
  - 多个文件头声明许可条款应见该软件组件根目录随附的 `LICENSE` 文件，但本仓库根目录没有一个可确认统一适用于这些生成文件的许可文件
  - 仓库已有 ST 组件许可文件，例如 [`Drivers/STM32F1xx_HAL_Driver/LICENSE.txt`](Drivers/STM32F1xx_HAL_Driver/LICENSE.txt) 与 [`Drivers/CMSIS/Device/ST/STM32F1xx/LICENSE.txt`](Drivers/CMSIS/Device/ST/STM32F1xx/LICENSE.txt)，但本清单不据此宣称它们覆盖 `Core/`、启动文件或其他 CubeMX 生成内容；具体适用范围待核实

- ALIENTEK MPU6050 / 软件 I2C 相关文件：
  - `Core/Inc/mpu6050.h`
  - `Core/Src/mpu6050.c`
  - `Core/Inc/mpuiic.h`
  - `Core/Src/mpuiic.c`
  - `Core/Src/inv_mpu.c` 中另有 ALIENTEK 标记的移植/附加部分
- OLED 相关文件：
  - `Core/Inc/oled.h`
  - `Core/Src/oled.c`
  - 文件头含来源版权信息及 `All rights reserved`，具体授权范围待来源方材料核实
- InvenSense 驱动：
  - `Core/Inc/inv_mpu.h`
  - `Core/Src/inv_mpu.c`
  - `Core/Inc/inv_mpu_dmp_motion_driver.h`
  - `Core/Src/inv_mpu_dmp_motion_driver.c`
  - 上述文件头引用了随附的 `License.txt`，但当前仓库未找到该许可文件
  - `Core/Inc/dmpKey.h` 与 `Core/Inc/dmpmap.h` 也带有 InvenSense 版权声明，应一并核实来源与授权范围

在公开分发、商业使用或更换许可前，建议从原始供应商或上游包取得与具体版本匹配的许可文本，并确认文件间的派生、移植和修改关系。不要删除现有源文件声明，也不要用一个新增的全仓许可文件覆盖这些未决边界。
