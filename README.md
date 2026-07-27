# Signal_AC_DC

基于 STM32G431C8T6 的单相 AC/DC 整流控制项目
内环控制频率为20k，外环控制频率为1k
TIM8不仅生成spwm波，而且负责作为ADC启动控制信号

## 功能

- SOGI-PLL 单相锁相
- PID-QPR  电压外环与电流内环
- SPWM 调制
- CMake + VS Code 构建

## 硬件平台

- MCU：STM32G431
- 控制频率：20 kHz
- 开发框架：STM32Cube HAL
- 构建工具：CMake、Ninja、arm-none-eabi-gcc
- 开发环境：VS Code

## 核心部分

- `Core/`：CubeMX 生成的核心代码
- `SOGI/`：SOGI-PLL 算法
- `control/`：PID、QPR 等控制器
- `pfc_spwm/`：SPWM 和功率级控制
- `cmake/`：CMake 工具链配置

## 编译

- openocd 进行编译控制

## 注意

- 本工程项目只能用于借鉴，不建议用于直接实践，只在低电压进行过测试，并没有在高电压调试过参数
