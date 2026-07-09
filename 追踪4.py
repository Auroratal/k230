import time
import gc
from machine import UART
from machine import FPIOA
from media.sensor import *
from media.display import *
from media.media import *
import cv_lite

# ============================================================
# 一、电机通信协议基础参数
# ============================================================

CMD_TAIL = 0x6B  #每条命令最后都要加 0x6B。
CW = 0  #表示一个方向
CCW = 1  #表示另一个方向

ADDR_UP = 1  #俯仰轴电机的驱动器地址，这里设为 1。
ADDR_DOWN = 2  #水平轴电机的驱动器地址，这里设为 2。

# ============================================================
# 二、电机方向配置
# ============================================================
# 下面四个变量决定：当目标偏到某个方向时，电机该往哪个方向转。
# 如果你发现追踪方向反了，不要改算法，只交换对应轴的 CW / CCW。
# ============================================================

DOWN_POSITIVE_DIR = CCW  # 水平轴误差为正时使用的方向；error_x > 0 表示目标在画面右边。
DOWN_NEGATIVE_DIR = CW  # 水平轴误差为负时使用的方向；error_x < 0 表示目标在画面左边。
UP_POSITIVE_DIR = CCW  # 俯仰轴误差为正时使用的方向；error_y > 0 表示目标在画面下边。
UP_NEGATIVE_DIR = CW  # 俯仰轴误差为负时使用的方向；error_y < 0 表示目标在画面上边。

# ============================================================
# 三、追踪控制参数
# ============================================================
# 这些参数决定云台追踪时“快不快、稳不稳、抖不抖”。
# 如果你后面要调效果，大部分都只需要改这里。
# ============================================================

TRACK_MAX_RPM_X = 30  # 水平轴最大转速，单位 RPM；越大追得越快，但越容易冲过头。
TRACK_MAX_RPM_Y = 25  # 俯仰轴最大转速，单位 RPM；通常俯仰轴可以比水平轴稍慢一些。

TRACK_MIN_RPM = 3  # 最小有效转速；太小步进电机可能不动，所以不要设得太低。

RPM_GAIN_X = 0.045  # 水平轴比例系数：像素误差乘它，变成电机转速。
RPM_GAIN_Y = 0.040  # 俯仰轴比例系数：像素误差乘它，变成电机转速。

DEADBAND_X = 36  # 水平轴死区，目标离中心小于 36 像素时就不转，防止来回抖。
DEADBAND_Y = 36  # 俯仰轴死区，目标离中心小于 36 像素时就不转，防止上下抖。

CONTROL_INTERVAL_MS = 30  # 控制周期，单位毫秒；每隔 30ms 才给电机更新一次速度。

RPM_STEP_X = 1  # 水平轴每次最多改变 1 RPM，让速度变化更平滑。
RPM_STEP_Y = 1  # 俯仰轴每次最多改变 1 RPM，让速度变化更平滑。

ACCEL = 20  # 驱动器加速度档位；比 0 更柔和，减少突然窜动。

CENTER_FILTER_ALPHA = 0.50  # 目标中心滤波系数；越大越跟手，越小越稳但延迟更大。

LOST_STOP_DELAY_MS = 60  # 目标丢失后等待 60ms 再停车，避免偶尔漏检一帧就急停。

TARGET_STABLE_FRAMES = 3  # 连续识别到 3 帧目标后才开始追踪，防止误检导致乱转。

EDGE_STOP_MARGIN_X = 70  # 目标距离左右边缘小于 70 像素时停车，防止目标被继续推出画面。
EDGE_STOP_MARGIN_Y = 55  # 目标距离上下边缘小于 55 像素时停车，防止目标被继续推出画面。

# ============================================================
# 四、图像尺寸和矩形识别参数
# ============================================================

W = 640  # 摄像头输出画面宽度，单位像素。
H = 480  # 摄像头输出画面高度，单位像素。
IMG = [H, W]  # cv_lite 检测函数需要的尺寸格式是 [高度, 宽度]。
TARGET_X = W // 2  # 画面中心 x 坐标，640 / 2 = 320。
TARGET_Y = H // 2  # 画面中心 y 坐标，480 / 2 = 240。

CANNY_LO = 50  # Canny 边缘检测低阈值，影响矩形边缘检测灵敏度。
CANNY_HI = 150  # Canny 边缘检测高阈值，影响矩形边缘检测灵敏度。
EPSILON = 0.08  # 多边形拟合精度，数值影响矩形边缘拟合结果。
AREA_MIN = 0.001  # cv_lite 内部面积过滤参数，太小的区域会忽略。
ANGLE_COS = 0.6  # 角度过滤参数，用来判断四边形角度是否像矩形。
BLUR_SIZE = 5  # 模糊核大小，用于降低图像噪声。

PAPER_W_CM = 20.0  # A4 纸实际宽度，单位厘米。
TARGET_R_CM = 6.0  # 画在靶心附近的圆半径，单位厘米，仅用于显示辅助。

RECT_AREA_MIN_PX = 12000  # 矩形最小面积，太小可能是噪声或远处杂物。
RECT_AREA_MAX_PX = 260000  # 矩形最大面积，太大可能是误识别到画面边框。
RECT_RATIO_MIN = 0.35  # 矩形宽高比下限；A4 倾斜后可能变窄，所以不能太严格。
RECT_RATIO_MAX = 2.80  # 矩形宽高比上限；A4 倾斜后可能变宽，所以不能太严格。

CONTINUITY_WEIGHT = 0.80  # 目标连续性权重；让程序更愿意选择靠近上一帧目标的矩形。

DEBUG_UART = False  # 是否打印每条串口命令；True 会很吵，也可能让画面变卡。
PRINT_EVERY_N_FRAMES = 30  # 每隔 30 帧打印一次状态，不要每帧都打印。
GC_EVERY_N_FRAMES = 90  # 每隔 90 帧做一次垃圾回收，不要每帧都 gc，否则会卡。

# ============================================================
# 五、UART3 串口初始化
# ============================================================
# 当前接线：
# K230 GPIO50 / UART3_TXD -> 驱动板 RX
# K230 GPIO51 / UART3_RXD <- 驱动板 TX
# K230 GND               -> 驱动板 GND
# ============================================================

fpioa = FPIOA()  # 创建 FPIOA 对象，用来配置 K230 引脚功能。
fpioa.set_function(50, FPIOA.UART3_TXD)  # 把 GPIO50 设置成 UART3 的发送引脚 TX。
fpioa.set_function(51, FPIOA.UART3_RXD)  # 把 GPIO51 设置成 UART3 的接收引脚 RX。

uart = UART( 
    UART.UART3, 
    baudrate=115200,  # 波特率 115200
    bits=UART.EIGHTBITS,  # 数据位 8 位
    parity=UART.PARITY_NONE,  # 无校验位
    stop=UART.STOPBITS_ONE,  # 1 个停止位
)  

# ============================================================
# 六、通用工具函数
# ============================================================


def sleep_ms(ms):  # 定义毫秒延时函数，参数 ms 表示延时多少毫秒。
    #兼容不同 MicroPython 版本的毫秒延时。
    if hasattr(time, "sleep_ms"):  # 判断 time 模块里有没有 sleep_ms 这个函数。
        time.sleep_ms(ms)  # 如果有，就直接用毫秒延时。
    else:  # 如果没有 sleep_ms，说明当前环境可能只支持秒级 sleep。
        time.sleep(ms / 1000.0)  # 把毫秒转换成秒，再延时。


def now_ms():  # 定义获取当前毫秒时间的函数。
    #获取当前时间，单位毫秒。
    if hasattr(time, "ticks_ms"):  # 判断 time 模块里有没有 ticks_ms。
        return time.ticks_ms() 
    return int(time.time() * 1000)  # 如果没有，就用秒级 time 转成毫秒。


def diff_ms(new, old): 
    #计算两个毫秒时间的差值。
    if hasattr(time, "ticks_diff"):  # 判断是否支持 ticks_diff。
        return time.ticks_diff(new, old)  
    return new - old  # 如果没有 ticks_diff，就普通相减。


def clamp(value, low, high):  # 定义限制范围函数，value 会被夹在 low 和 high 之间。
    if value < low: 
        return low 
    if value > high:
        return high  
    return value 


def approach(current, target, step):  # 定义平滑靠近函数，防止速度突然跳变。
    #让 current 慢慢靠近 target，每次最多变化 step。
    if current < target:
        return min(current + step, target)  # 增加 step，但不能超过目标值。
    if current > target: 
        return max(current - step, target)  # 减少 step，但不能低于目标值。
    return current  # 如果已经等于目标值，就不变。

# ============================================================
# 七、电机串口命令函数
# ============================================================


def send_cmd(addr, payload):  # 定义发送命令函数，addr 是电机地址，payload 是命令内容。
    #发送一条命令给指定地址的驱动器。
    frame = bytes([addr] + payload + [CMD_TAIL])  # 拼成完整帧：地址 + 命令 + 结尾 0x6B。
    uart.write(frame)  # 通过 UART3 把这一帧发送给驱动器。
    if DEBUG_UART:  # 如果开启串口调试输出。
        print("send:", [hex(x) for x in frame])  # 把发送的每个字节以 16 进制打印出来。


def enable_motor(addr):  # 定义使能电机函数。
    send_cmd(addr, [0xF3, 0xAB, 0x01, 0x00])  # 发送使能命令


def stop_motor(addr):  # 定义停止电机函数。
    #停止电机。速度模式下非常重要。
    send_cmd(addr, [0xFE, 0x98, 0x00])  # 发送 Emm_V5 停止命令：FE 98 00。


def velocity_move(addr, direction, speed_rpm):  # 定义速度模式运动函数。
    #速度模式控制电机
    speed_rpm = clamp(int(speed_rpm), 0, 5000)  # 把速度转成整数，并限制在 0 到 5000 RPM。
    send_cmd(addr, [  # 发送 F6 速度模式命令。
        0xF6,  # 速度模式。
        direction & 0xFF,  # 方向字节，只保留低 8 位。
        (speed_rpm >> 8) & 0xFF,  # 速度高 8 位。
        speed_rpm & 0xFF,  # 速度低 8 位。
        ACCEL & 0xFF,  # 加速度档位，只保留低 8 位。
        0x00,  # 同步标志，这里 0 表示不使用同步。
    ]) 

# ============================================================
# 八、电机状态变量
# ============================================================
# 这些变量是程序运行时用来记住“电机上一次是什么状态”。
# 记住状态后，程序就可以少发重复命令，也能判断什么时候需要停。
# ============================================================

last_x_dir = None  # 水平轴上一次发送的方向；None 表示还没发过。
last_y_dir = None  # 俯仰轴上一次发送的方向；None 表示还没发过。
last_x_rpm_sent = -1  # 水平轴上一次发送的速度；-1 表示还没发过。
last_y_rpm_sent = -1  # 俯仰轴上一次发送的速度；-1 表示还没发过。
x_rpm_current = 0  # 水平轴当前内部记录速度，用于平滑变化。
y_rpm_current = 0  # 俯仰轴当前内部记录速度，用于平滑变化。
x_running = False  # 水平轴当前是否被认为正在转。
y_running = False  # 俯仰轴当前是否被认为正在转。


def stop_axis(addr, axis_name):  # 定义停止某一个轴的函数。
    #停止单个轴，并清空这个轴的状态记录
    global last_x_dir, last_y_dir, last_x_rpm_sent, last_y_rpm_sent  # 声明要修改全局方向和速度记录。
    global x_rpm_current, y_rpm_current, x_running, y_running  # 声明要修改全局运行状态。

    stop_motor(addr)  # 给驱动器发送停止命令。
    if axis_name == "x":  # 如果要停的是水平轴,清空水平轴方向记录,把水平轴发送速度记录为 0,把水平轴内部速度清零,标记水平轴不再运行。
        last_x_dir = None
        last_x_rpm_sent = 0
        x_rpm_current = 0
        x_running = False
    else:  # 如果要停的不是水平轴，那就是俯仰轴,清空俯仰轴方向记录,
        #把俯仰轴发送速度记录为 0,把俯仰轴内部速度清零,标记俯仰轴不再运行。
        last_y_dir = None
        last_y_rpm_sent = 0
        y_rpm_current = 0
        y_running = False


def stop_all_motors():  # 定义停止两个电机的函数。
    stop_axis(ADDR_DOWN, "x")  # 停止水平轴
    sleep_ms(5)  # 两条停止命令之间隔5ms
    stop_axis(ADDR_UP, "y")  # 停止俯仰轴


def rpm_from_error(error, deadband, gain, max_rpm):  # 定义误差转速度函数。
    #把像素误差转换成电机转速。
    if abs(error) <= deadband:  # 如果误差绝对值小于死区。
        return 0  #不用转。
    rpm = int((abs(error) - deadband) * gain)  # 误差减去死区后乘比例系数，得到目标转速。
    return clamp(rpm, TRACK_MIN_RPM, max_rpm)  # 把转速限制在最小转速和最大转速之间。


def update_axis_speed(addr, axis_name, error, deadband, gain, max_rpm, rpm_step, positive_dir, negative_dir):  # 更新单个轴速度。
    #根据某个轴的误差，更新这个轴的电机速度。
    global last_x_dir, last_y_dir, last_x_rpm_sent, last_y_rpm_sent  
    # 声明要修改全局方向/速度记录。
    global x_rpm_current, y_rpm_current, x_running, y_running  
    # 声明要修改全局运行状态。

    target_rpm = rpm_from_error(error, deadband, gain, max_rpm)  # 根据像素误差算出目标转速。

    if axis_name == "x":  # 如果当前更新的是水平轴,取出水平轴当前内部速度,取出水平轴上一次方向,
        #取出水平轴上一次发送速度,取出水平轴运行状态。
        current_rpm = x_rpm_current
        last_dir = last_x_dir
        last_sent = last_x_rpm_sent
        running = x_running
    else:  # 否则当前更新的是俯仰轴,取出俯仰轴当前内部速度,取出俯仰轴上一次方向,
           # 取出俯仰轴上一次发送速度,取出俯仰轴运行状态。
        current_rpm = y_rpm_current
        last_dir = last_y_dir
        last_sent = last_y_rpm_sent
        running = y_running

    if target_rpm == 0:  # 如果目标转速是 0，说明目标进入死区或误差很小。
        if running:  # 如果这个轴之前还在运行。
            stop_axis(addr, axis_name)  # 立刻停
        return 0  # 返回实际速度 0。

    direction = positive_dir if error > 0 else negative_dir  
    # 根据误差正负选择电机方向。

    if running and last_dir is not None and direction != last_dir: 
         # 如果正在转，且方向和上一次不同，先停一下，避免直接反向,
         #等3ms,内部速度清零，从零开始重新加速,标记当前轴暂时不运行,
         #上一次发送速度重置为 -1，强制下一步发送新命令。
        stop_axis(addr, axis_name)  
        sleep_ms(3)
        current_rpm = 0
        running = False
        last_sent = -1

    current_rpm = approach(current_rpm, target_rpm, rpm_step)  # 让当前速度逐步靠近目标速度。
    rpm = clamp(int(current_rpm), TRACK_MIN_RPM, max_rpm)  # 把当前速度转成整数，并限制范围。

    if direction != last_dir or abs(rpm - last_sent) >= 1 or not running:  
        # 判断是否需要发新速度命令。
        velocity_move(addr, direction, rpm)  # 给驱动器发送速度模式命令。
        last_dir = direction  # 更新“上一次方向”。
        last_sent = rpm  # 更新“上一次发送速度”。
        running = True  # 标记这个轴正在运行。

    if axis_name == "x":  # 如果刚才更新的是水平轴,保存水平轴当前内部速度,
        # 保存水平轴上一次方向,保存水平轴上一次发送速度,保存水平轴运行状态。
        x_rpm_current = current_rpm 
        last_x_dir = last_dir
        last_x_rpm_sent = last_sent
        x_running = running
    else:  # 如果刚才更新的是俯仰轴,保存俯仰轴当前内部速度,
         # 保存俯仰轴上一次方向,保存俯仰轴上一次发送速度,保存俯仰轴运行状态,
        y_rpm_current = current_rpm  
        last_y_dir = last_dir
        last_y_rpm_sent = last_sent  
        y_running = running
        
    # 返回这次实际使用的速度，方便打印调试。

    return rpm 

def track_target(cx, cy):  # 定义追踪目标函数，输入目标中心坐标。
    #根据目标中心点，分别控制水平轴和俯仰轴
    error_x = int(cx) - TARGET_X  # 目标 x 坐标减画面中心 x，得到水平误差。
    error_y = int(cy) - TARGET_Y  # 目标 y 坐标减画面中心 y，得到俯仰误差。

    rpm_x = update_axis_speed(  # 根据水平误差更新水平轴速度。
        ADDR_DOWN,  # 水平轴使用地址 2 的电机。
        "x",  # 轴名字写 x，用于更新 x 轴状态变量。
        error_x,  # 水平误差。
        DEADBAND_X,  # 水平死区。
        RPM_GAIN_X,  # 水平比例系数。
        TRACK_MAX_RPM_X,  # 水平最大速度。
        RPM_STEP_X,  # 水平速度变化步长。
        DOWN_POSITIVE_DIR,  # 水平误差为正时的方向。
        DOWN_NEGATIVE_DIR,  # 水平误差为负时的方向。
    )

    rpm_y = update_axis_speed(  # 根据垂直误差更新俯仰轴速度。
        ADDR_UP,  # 俯仰轴使用地址 1 的电机。
        "y",  # 轴名字写 y，用于更新 y 轴状态变量。
        error_y,  # 垂直误差。
        DEADBAND_Y,  # 垂直死区。
        RPM_GAIN_Y,  # 垂直比例系数。
        TRACK_MAX_RPM_Y,  # 垂直最大速度。
        RPM_STEP_Y,  # 垂直速度变化步长。
        UP_POSITIVE_DIR,  # 垂直误差为正时的方向。
        UP_NEGATIVE_DIR,  # 垂直误差为负时的方向。
    )

    return error_x, error_y, rpm_x, rpm_y  # 返回误差和速度，方便主循环打印。

# ============================================================
# 九、视觉处理函数
# ============================================================


def draw_quad(img, r, color):  # 定义画四边形函数，img 是图像，r 是矩形信息，color 是颜色。
    """把识别出来的四边形画在画面上。"""  # 函数说明。
    pts = [(r[4], r[5]), (r[6], r[7]), (r[8], r[9]), (r[10], r[11])]  # 从 r 中取出四个角点。
    for i in range(4):  # 循环 4 次，依次连接四条边。
        x1, y1 = pts[i]  # 当前角点。
        x2, y2 = pts[(i + 1) % 4]  # 下一个角点，最后一个点会连回第一个点。
        img.draw_line(x1, y1, x2, y2, color=color)  # 在图像上画一条边。


def diagonal_center(r):  # 定义计算矩形中心的函数。
    """计算矩形中心：用两条对角线的交点。"""  # 函数说明。
    x1, y1 = r[4], r[5]  # 第 1 个角点。
    x2, y2 = r[8], r[9]  # 第 3 个角点，和第 1 个角点组成一条对角线。
    x3, y3 = r[6], r[7]  # 第 2 个角点。
    x4, y4 = r[10], r[11]  # 第 4 个角点，和第 2 个角点组成另一条对角线。

    d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)  # 两条直线求交点公式里的分母。
    if abs(d) < 0.001:  # 如果分母很小，说明两条线几乎平行，交点不好算。
        return (x1 + x2 + x3 + x4) // 4, (y1 + y2 + y3 + y4) // 4  # 退而求其次，用四个角点平均值。

    t = float((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / d  # 求交点参数 t。
    cx = int(x1 + t * (x2 - x1))  # 根据 t 算出交点 x 坐标。
    cy = int(y1 + t * (y2 - y1))  # 根据 t 算出交点 y 坐标。
    return cx, cy  # 返回矩形中心坐标。


def is_valid_rect(r):  # 定义矩形有效性判断函数。
    #判断一个矩形是不是可能的目标靶纸
    area = r[2] * r[3]  # 用矩形外接框宽度乘高度，估算面积。
    if area < RECT_AREA_MIN_PX or area > RECT_AREA_MAX_PX:  # 如果面积太小或太大。
        return False  # 认为不是目标。

    h = r[3]  # 取出矩形外接框高度。
    if h <= 0:  # 如果高度不正常。
        return False  # 认为不是目标。

    ratio = r[2] / h  # 计算宽高比。
    if ratio < RECT_RATIO_MIN or ratio > RECT_RATIO_MAX:  # 如果宽高比超出允许范围。
        return False  # 认为不是目标。

    return True  # 面积和宽高比都合格，认为可能是目标。


def choose_best_rect(rects, last_cx, last_cy):  # 定义从多个矩形里选择最佳目标的函数。
    #从所有矩形里选出最适合追踪的那个。
    best = None  # 当前最好的矩形，开始时没有。
    best_score = -999999999  # 当前最高分，先给一个很小的数。

    for i in range(len(rects)):  # 遍历检测到的所有矩形。
        r = rects[i]  # 取出第 i 个矩形。
        if not is_valid_rect(r):  # 如果这个矩形不符合面积/比例要求。
            continue  # 跳过这个矩形，看下一个。

        area = r[2] * r[3]  # 计算这个矩形面积。
        cx, cy = diagonal_center(r)  # 计算这个矩形中心。
        score = area  # 初始分数等于面积，面积越大越可能是目标。

        if last_cx is not None and last_cy is not None:  # 如果上一帧有目标位置。
            dx = cx - last_cx  # 当前矩形中心和上一帧目标中心的 x 距离。
            dy = cy - last_cy  # 当前矩形中心和上一帧目标中心的 y 距离。
            dist2 = dx * dx + dy * dy
            score = area - int(dist2 * CONTINUITY_WEIGHT)  # 离上一帧越远，扣分越多。

        if score > best_score:  # 如果当前矩形分数比之前最好的更高。
            best_score = score  # 更新最高分。
            best = r  # 把当前矩形记为最佳目标。

    return best  # 返回最佳矩形；如果没有合格矩形，就返回 None。


def near_image_edge(cx, cy):  # 定义判断目标是否靠近画面边缘的函数。
    #判断目标中心是不是太靠近画面边缘。
    return ( 
        cx < EDGE_STOP_MARGIN_X 
        or
        cx > W - EDGE_STOP_MARGIN_X 
        or
        cy < EDGE_STOP_MARGIN_Y 
        or
        cy > H - EDGE_STOP_MARGIN_Y
    ) 

# ============================================================
# 十、摄像头和显示初始化
# ============================================================

sensor = Sensor(id=1, width=1280, height=960, fps=90)  # 创建摄像头对象
sensor.reset()  # 重置摄像头
sensor.set_framesize(width=W, height=H)  # 设置图像大小为 640x480。
sensor.set_pixformat(Sensor.RGB888)  # 设置像素格式为 RGB888。

try:  # 设置摄像头增益。
    g = k_sensor_gain()  # 创建/读取增益配置对象。
    g.gain[0] = 20  # 设置第 0 路增益为 20，提高画面亮度。
    sensor.again(g)  # 把增益配置写入摄像头。
except Exception:  # 如果固件不支持或设置失败,忽略错误，不影响主程序继续运行。
    pass  

Display.init(Display.ST7701, width=W, height=H, to_ide=True, quality=50)  # 初始化显示
MediaManager.init()
sensor.run()
sleep_ms(200)

# ============================================================
# 十一、程序启动前准备
# ============================================================

clock = time.clock()  # 创建 FPS 计时器。
print("K230 UART3 anti-drift tracking start")  # 打印程序启动信息。
print("camera = CSI1 / Sensor id=1")  # 打印当前摄像头接口信息。
print("UART = UART3 GPIO50/GPIO51")  # 打印当前串口引脚信息。
print("up addr =", ADDR_UP, "down addr =", ADDR_DOWN)  # 打印两个电机地址。

print("enable motors")  # 打印正在使能电机。
enable_motor(ADDR_UP)  # 使能地址 1 电机。
sleep_ms(80)  # 等 80ms，让驱动器处理命令。
enable_motor(ADDR_DOWN)  # 使能地址 2 电机。
sleep_ms(120)  # 等 120ms，让驱动器处理命令。
stop_all_motors()  # 先停一次，确保启动时两个电机不是速度模式残留状态。

last_control_ms = now_ms()  # 记录上一次控制电机的时间。
last_seen_ms = now_ms()  # 记录上一次看到目标的时间。
frame_count = 0  # 帧计数器，从 0 开始。
stable_target_count = 0  # 连续稳定识别到目标的帧数。
filtered_cx = None  # 滤波后的目标中心 x，开始时没有目标，所以是 None。
filtered_cy = None  # 滤波后的目标中心 y，开始时没有目标，所以是 None。
last_target_cx = None  # 上一帧目标中心 x，用于目标连续性选择。
last_target_cy = None  # 上一帧目标中心 y，用于目标连续性选择。
target_active = False  # 当前是否处于追踪状态。
motors_stopped_after_lost = True  # 目标丢失后是否已经停车。

# ============================================================
# 十二、主循环
# ============================================================

try:  # try 用来保证程序退出时可以进入 finally 停车和释放资源。
    while True:  # 无限循环，程序会一直追踪。
        frame_count += 1  # 帧计数加 1。
        t_now = now_ms()  # 记录当前时间，单位毫秒。
        clock.tick()  # FPS 计时器更新一帧。

        img = sensor.snapshot()  # 从摄像头取一帧图像。
        np_img = img.to_numpy_ref()  # 把图像转成 cv_lite 可以直接处理的引用。

        rects = cv_lite.rgb888_find_rectangles_with_corners(  # 调用 cv_lite 检测矩形。
            IMG,  # 图像尺寸 [H, W]。
            np_img,  # 图像数据。
            CANNY_LO,  # Canny 低阈值。
            CANNY_HI,  # Canny 高阈值。
            EPSILON,  # 多边形拟合参数。
            AREA_MIN,  # 最小面积参数。
            ANGLE_COS,  # 角度过滤参数。
            BLUR_SIZE,  # 模糊核大小。
        )

        best = choose_best_rect(rects, last_target_cx, last_target_cy)  # 选出最适合追踪的矩形。
        info = "none"  # 默认调试信息是 none，表示暂时没有目标。

        if best:  # 如果找到了合格目标。
            raw_cx, raw_cy = diagonal_center(best)  # 计算目标原始中心点。
            last_target_cx = raw_cx  # 保存这一帧目标中心 x，供下一帧做连续性判断。
            last_target_cy = raw_cy  # 保存这一帧目标中心 y，供下一帧做连续性判断。
            last_seen_ms = t_now  # 更新最后看到目标的时间。
            motors_stopped_after_lost = False  # 既然看到目标了，就标记“不是丢失后已停车”。

            if filtered_cx is None:  # 如果滤波中心还没有初始值。
                filtered_cx = raw_cx  # 第一次直接用原始中心 x。
                filtered_cy = raw_cy  # 第一次直接用原始中心 y。
            else:  # 如果已经有滤波中心。
                filtered_cx = int(filtered_cx * (1.0 - CENTER_FILTER_ALPHA) + raw_cx * CENTER_FILTER_ALPHA)  # x 低通滤波。
                filtered_cy = int(filtered_cy * (1.0 - CENTER_FILTER_ALPHA) + raw_cy * CENTER_FILTER_ALPHA)  # y 低通滤波。

            stable_target_count += 1  # 连续识别到目标的帧数加 1。

            draw_quad(img, best, color=(0, 255, 255))  # 在画面上用黄色/青色线画出目标四边形。
            img.draw_cross(TARGET_X, TARGET_Y, color=(255, 0, 0), size=12, thickness=2)  # 在画面中心画红色十字。
            img.draw_cross(filtered_cx, filtered_cy, color=(0, 255, 0), size=15, thickness=3)  # 在目标中心画绿色十字。

            w_px = best[2]  # 取目标外接框宽度，单位像素。
            if w_px > 0:  # 如果宽度有效。
                r_px = int(TARGET_R_CM * (w_px / PAPER_W_CM))  # 根据 A4 宽度比例，算 6cm 圆的像素半径。
                img.draw_circle(filtered_cx, filtered_cy, r_px, color=(0, 255, 0), thickness=2)  # 在目标中心画 6cm 辅助圆。

            err_x = filtered_cx - TARGET_X  # 计算水平误差：目标中心 x - 画面中心 x。
            err_y = filtered_cy - TARGET_Y  # 计算垂直误差：目标中心 y - 画面中心 y。
            rpm_x = 0  # 默认水平轴速度为 0。
            rpm_y = 0  # 默认俯仰轴速度为 0。

            if diff_ms(t_now, last_control_ms) >= CONTROL_INTERVAL_MS:  # 如果距离上次控制已经超过控制周期。
                last_control_ms = t_now  # 更新上次控制时间。

                if near_image_edge(filtered_cx, filtered_cy):  # 如果目标已经接近画面边缘。
                    stop_all_motors()  # 立刻停止两个电机
                    target_active = False  # 标记当前不再追踪。
                    motors_stopped_after_lost = True  # 标记电机已经停过。
                    info = "edge stop"  # 调试信息写成 edge stop。
                elif stable_target_count >= TARGET_STABLE_FRAMES:  # 如果目标已经连续稳定出现足够多帧。
                    target_active = True  # 标记当前处于追踪状态。
                    err_x, err_y, rpm_x, rpm_y = track_target(filtered_cx, filtered_cy)  # 根据目标中心控制电机。

            info = "center=({}, {}) err=({}, {}) rpm=({}, {}) stable={}".format(  # 生成调试字符串。
                filtered_cx, filtered_cy, err_x, err_y, rpm_x, rpm_y, stable_target_count)  # 填入中心、误差、速度、稳定帧数。

        else:  # 如果没有找到目标。
            stable_target_count = 0  # 连续识别计数清零。
            # 清空滤波中心 x,y。
            filtered_cx = None  
            filtered_cy = None  

            if target_active and not motors_stopped_after_lost:  # 如果之前正在追踪，而且丢目标后还没停过车。
                if diff_ms(t_now, last_seen_ms) >= LOST_STOP_DELAY_MS:  # 如果丢目标时间超过允许延迟。
                    stop_all_motors()  # 停止两个电机。
                    motors_stopped_after_lost = True  # 标记已经停车。
                    target_active = False  # 标记不再追踪。
                    last_target_cx = None  # 清空上一帧目标 x，避免下次误用旧目标。
                    last_target_cy = None  # 清空上一帧目标 y，避免下次误用旧目标。

        fps = clock.fps()  # 计算当前 FPS。
        img.draw_string_advanced(10, 5, 22, "FPS:{:.1f}".format(fps), color=(0, 255, 0))  # 在画面左上角显示 FPS。
        Display.show_image(img)  # 把处理后的图像显示到 CanMV IDE / 屏幕。

        if frame_count % PRINT_EVERY_N_FRAMES == 0:  # 如果到了打印间隔。
            print("FPS:{:.1f} rects:{} {}".format(fps, len(rects), info))  # 打印 FPS、矩形数量和调试信息。

        if frame_count % GC_EVERY_N_FRAMES == 0:  # 如果到了垃圾回收间隔。
            gc.collect()  # 主动回收内存，减少长时间运行出问题的概率。

except KeyboardInterrupt:  # 如果用户手动停止程序。
    print("user stop") 

finally:  # 无论正常退出还是报错退出，都会执行这里。
    try:  # 尝试停车。
        stop_all_motors()  # 停止两个电机，防止程序停了电机还在转。
    except Exception:  # 如果停车时出错。
        pass  # 忽略错误，继续做后面的清理。

    try: 
        sensor.stop()
    except Exception:
        pass

    try:
        Display.deinit()
    except Exception:
        pass 

    try:
        MediaManager.deinit()
    except Exception:
        pass
