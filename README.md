# K230 云台矩形追踪程序说明

本文档对应文件：`追踪4.py`

这份代码用于庐山派 K230 / CanMV，通过摄像头识别画面中的矩形靶纸，然后用 UART3 串口控制两个 Emm_V5 / 张大头闭环步进电机，让云台自动把矩形中心追到画面中心。

## 一句话理解

程序一直做这件事：

```text
摄像头拍图 -> 找矩形 -> 算矩形中心 -> 和画面中心比较 -> 控制两个电机转动 -> 让矩形回到画面中心
```

它是一个视觉闭环控制程序：

```text
摄像头负责看偏差
程序负责算偏差
电机负责把偏差转回来
```

## 硬件连接

当前代码默认使用 UART3。

K230 侧：

```text
GPIO50 -> UART3_TXD
GPIO51 -> UART3_RXD
GND    -> GND
```

连接驱动器：

```text
K230 GPIO50 / UART3_TXD -> 驱动板 RX
K230 GPIO51 / UART3_RXD <- 驱动板 TX，可选
K230 GND                -> 驱动板 GND，必须共地
```

注意：

```text
TX 接对方 RX
RX 接对方 TX
GND 必须共地
```

如果只是发送控制命令，不读取驱动器回复，可以先只接：

```text
K230 TX -> 驱动板 RX
K230 GND -> 驱动板 GND
```

## 驱动器地址

代码里默认：

```python
ADDR_UP = 1
ADDR_DOWN = 2
```

含义：

```text
地址 1：上方 / 俯仰轴电机
地址 2：下方 / 水平轴电机
```

驱动器菜单里的 `ID_ADDR` 必须和代码一致。

如果你的两个驱动器地址不是 1 和 2，需要改这两行。

## 摄像头接口

代码里默认：

```python
sensor = Sensor(id=1, width=1280, height=960, fps=90)
```

说明当前摄像头接在 `CSI1`。

如果以后你把摄像头换到别的 CSI 接口，才需要改 `id`。

## 串口参数

代码里默认：

```python
baudrate=115200
bits=UART.EIGHTBITS
parity=UART.PARITY_NONE
stop=UART.STOPBITS_ONE
```

也就是常见的：

```text
115200, 8N1
```

驱动器菜单里的串口波特率也要是 `115200`。

## 主要参数解释

### 最大转速

```python
TRACK_MAX_RPM_X = 30
TRACK_MAX_RPM_Y = 25
```

控制云台最快能转多快。

如果追踪太慢，可以稍微调大。

如果目标容易冲出画面，可以调小。

### 最小转速

```python
TRACK_MIN_RPM = 3
```

电机太低速可能不动，所以设置一个最小有效转速。

如果电机小误差时不动，可以稍微调大。

如果中心附近动作太突兀，可以稍微调小，但太小可能没反应。

### 比例系数

```python
RPM_GAIN_X = 0.045
RPM_GAIN_Y = 0.040
```

它决定“偏差变成速度”的比例。

```text
偏差越大 -> 速度越大
gain 越大 -> 反应越猛
```

如果追踪太慢，可以调大一点。

如果追踪过冲、乱飘、甩出画面，可以调小一点。

### 死区

```python
DEADBAND_X = 36
DEADBAND_Y = 36
```

死区就是“目标已经够靠近中心了，就别动”。

例如水平死区是 36：

```text
目标离画面中心小于 36 像素 -> 水平轴不转
```

如果中心附近一直抖，把死区调大。

如果对准精度不够，把死区调小。

### 控制周期

```python
CONTROL_INTERVAL_MS = 30
```

表示每隔 30ms 才更新一次电机速度。

太小：命令太频繁，可能卡或抖。

太大：追踪反应慢。

### 速度变化步长

```python
RPM_STEP_X = 1
RPM_STEP_Y = 1
```

表示每次控制时，速度最多变化多少 RPM。

这个用来让速度平滑变化。

例如当前速度是 3，目标速度是 10，步长是 1，那么速度会变成：

```text
3 -> 4 -> 5 -> 6 ...
```

而不是一下子从 3 跳到 10。

### 加速度档位

```python
ACCEL = 20
```

这是发给驱动器的加速度参数。

如果启动太冲，可以适当增大。

如果反应太慢，可以减小。

### 中心滤波

```python
CENTER_FILTER_ALPHA = 0.50
```

摄像头识别到的目标中心会抖动，滤波可以让中心点更稳定。

值越大：越跟手，但越容易抖。

值越小：越稳定，但延迟更大。

### 丢目标停车时间

```python
LOST_STOP_DELAY_MS = 60
```

目标丢失超过 60ms 后停车。

不是一丢就停，是因为视觉可能偶尔漏检一帧。

### 稳定帧数

```python
TARGET_STABLE_FRAMES = 3
```

连续识别到 3 帧目标后才开始追踪。

这样可以避免误识别到一帧杂物就乱动。

### 边缘保护

```python
EDGE_STOP_MARGIN_X = 70
EDGE_STOP_MARGIN_Y = 55
```

如果目标太靠近画面边缘，就停车。

作用是防止继续追踪时把目标甩出画面。

## 每个函数的作用

### `sleep_ms(ms)`

延时函数。

让程序暂停指定毫秒数。

### `now_ms()`

获取当前时间，单位毫秒。

用于判断控制周期、丢目标时间等。

### `diff_ms(new, old)`

计算两个时间之间差了多少毫秒。

例如判断是否该更新电机速度。

### `clamp(value, low, high)`

把一个数限制在指定范围内。

例如限制电机速度不能太小，也不能太大。

### `approach(current, target, step)`

让 `current` 慢慢靠近 `target`。

用于让电机速度平滑变化，避免突然猛冲。

### `send_cmd(addr, payload)`

串口发送底层函数。

所有发给电机驱动器的命令最后都通过它发送。

它会自动拼接：

```text
电机地址 + 命令内容 + 0x6B
```

### `enable_motor(addr)`

使能指定地址的电机。

电机不使能时，运动命令可能不执行。

### `stop_motor(addr)`

停止指定地址的电机。

速度模式下很重要，因为速度模式会一直转，必须主动停。

### `velocity_move(addr, direction, speed_rpm)`

速度模式运动函数。

发出后，驱动器会让电机按指定方向和速度持续转动。

### `stop_axis(addr, axis_name)`

停止某一个轴，并清空这个轴的状态记录。

不仅发停止命令，还会把方向、速度、运行状态清掉。

### `stop_all_motors()`

停止两个电机。

内部会分别停止水平轴和俯仰轴。

### `rpm_from_error(error, deadband, gain, max_rpm)`

把像素误差转换成电机转速。

逻辑是：

```text
误差在死区内 -> 速度为 0
误差越大 -> 速度越大
速度不能超过最大值
```

### `update_axis_speed(...)`

单个电机轴的核心控制函数。

它负责：

```text
根据偏差算速度
判断是否进入死区
决定方向
处理反向切换
平滑速度
发送速度命令
保存轴状态
```

可以理解成“单个轴的大脑”。

### `track_target(cx, cy)`

根据目标中心点控制两个轴。

它会算：

```python
error_x = cx - TARGET_X
error_y = cy - TARGET_Y
```

然后分别调用 `update_axis_speed()` 控制水平轴和俯仰轴。

### `draw_quad(img, r, color)`

把识别到的矩形画在画面上。

用于调试显示，不直接控制电机。

### `diagonal_center(r)`

根据矩形四个角点计算中心点。

它用的是两条对角线交点，比直接用外接框中心更适合倾斜的纸面。

### `is_valid_rect(r)`

判断某个矩形是不是可能的目标。

主要检查面积和宽高比。

### `choose_best_rect(rects, last_cx, last_cy)`

从所有检测到的矩形中选出最适合追踪的目标。

选择标准：

```text
面积大
离上一帧目标位置近
```

### `near_image_edge(cx, cy)`

判断目标中心是否太靠近画面边缘。

如果太靠边，程序会停车，防止目标被甩出画面。

## 主循环思路

主循环是代码最重要的部分。

它一直重复执行：

```text
拍图 -> 找矩形 -> 选目标 -> 算中心 -> 控制电机 -> 显示画面
```

### 1. 拍一帧图像

```python
img = sensor.snapshot()
np_img = img.to_numpy_ref()
```

摄像头拍图，并转成 `cv_lite` 能处理的格式。

### 2. 检测矩形

```python
rects = cv_lite.rgb888_find_rectangles_with_corners(...)
```

找出画面中所有像矩形的目标。

### 3. 选择目标矩形

```python
best = choose_best_rect(rects, last_target_cx, last_target_cy)
```

从所有矩形中选一个最像靶纸的。

### 4. 如果找到了目标

程序会：

```text
算矩形中心
更新目标出现时间
对中心点滤波
画出矩形和中心点
计算目标和画面中心的误差
根据误差控制电机
```

核心控制入口是：

```python
track_target(filtered_cx, filtered_cy)
```

### 5. 如果目标靠近边缘

```python
if near_image_edge(filtered_cx, filtered_cy):
    stop_all_motors()
```

快到边缘就停，避免目标被继续推出画面。

### 6. 如果没找到目标

程序会清空目标状态。

如果目标丢失超过 `LOST_STOP_DELAY_MS`，就停止两个电机。

### 7. 显示画面

```python
Display.show_image(img)
```

把带有识别框、中心点、FPS 的画面显示出来。

### 8. 偶尔打印和回收内存

不是每帧打印，也不是每帧 `gc.collect()`，因为这样会卡。

## 整体调用链

```text
主循环 while True
    -> 摄像头 snapshot
    -> cv_lite 找矩形
    -> choose_best_rect 选目标
    -> diagonal_center 算中心
    -> track_target 追踪目标
    -> update_axis_speed 控制单个轴
    -> velocity_move 生成速度命令
    -> send_cmd 串口发送给驱动器
```

## 常见问题

### 目标在中心附近一直抖

把死区调大：

```python
DEADBAND_X = 45
DEADBAND_Y = 45
```

或者把比例系数调小：

```python
RPM_GAIN_X = 0.035
RPM_GAIN_Y = 0.030
```

### 追踪太慢

可以稍微调大最大转速：

```python
TRACK_MAX_RPM_X = 35
TRACK_MAX_RPM_Y = 30
```

或者稍微调大比例系数：

```python
RPM_GAIN_X = 0.055
RPM_GAIN_Y = 0.050
```

### 一移动就冲出画面

调小最大转速：

```python
TRACK_MAX_RPM_X = 20
TRACK_MAX_RPM_Y = 18
```

调小比例系数：

```python
RPM_GAIN_X = 0.030
RPM_GAIN_Y = 0.028
```

### 没有目标时电机还在转

重点检查：

```python
LOST_STOP_DELAY_MS
near_image_edge()
stop_all_motors()
```

如果还会继续转，可以把 `LOST_STOP_DELAY_MS` 调小。

### 追踪方向反了

只改方向配置：

```python
DOWN_POSITIVE_DIR = CCW
DOWN_NEGATIVE_DIR = CW
UP_POSITIVE_DIR = CCW
UP_NEGATIVE_DIR = CW
```

水平轴反了，交换 `DOWN_POSITIVE_DIR` 和 `DOWN_NEGATIVE_DIR`。

俯仰轴反了，交换 `UP_POSITIVE_DIR` 和 `UP_NEGATIVE_DIR`。

## 不建议乱改的地方

一般不要随便改：

```text
send_cmd()
velocity_move()
update_axis_speed()
diagonal_center()
cv_lite.rgb888_find_rectangles_with_corners(...)
```

调效果优先改参数，不要先改核心函数。

## 推荐调参顺序

如果追踪效果不好，建议按这个顺序调：

1. 先确认方向对不对。
2. 再调最大速度 `TRACK_MAX_RPM_X/Y`。
3. 再调比例系数 `RPM_GAIN_X/Y`。
4. 再调死区 `DEADBAND_X/Y`。
5. 最后才考虑改识别参数。

不要一次改很多参数，否则不知道是哪一个参数产生了效果。
