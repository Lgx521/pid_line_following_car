from motor_control import motor, PID, counter
from machine import Pin, PWM, Timer, I2C
import utime
from jy61p import jy61p_i2c


## 常数
LEFT = -1
RIGHT = 1

## 所有引脚定义
# I2C输入
OLED_SDA = 0
OLED_SCL = 1
GYRO_SDA = 2
GYRO_SCL = 3

# Motor1 所有控制
PWM_OUT_1 = 6
M1 = 7
M2 = 8
PWM_IN_1 = 21
PWM_IN_2 = 20

# Motor2 所有控制
PWM_OUT_2 = 10
M3 = 11
M4 = 12
PWM_IN_3 = 19
PWM_IN_4 = 18

# 外部中断
EXT_IRT_START = 16
EXT_IRT_INIT = 22

# 程序状态指示
STATUS_LED = 17

# 红外模块模拟输入
ANALOG_1 = 26
ANALOG_2 = 27
ANALOG_3 = 28

## 全局变量
# 电机编码器返回频率计算值
freq_1 = 0
freq_2 = 0

# 计数器（频率计算器）对象
counter1 = counter(PWM_IN_1)
counter2 = counter(PWM_IN_3)

# 电机控制计时器
timer_dutyset = Timer()

# 电机对象
m1 = motor(M1, M2, PWM_OUT_1, PWM_IN_1)
m2 = motor(M3, M4, PWM_OUT_2, PWM_IN_3)

# 电机编码器返回频率目标
target_1 = 0
target_2 = 0

# 角度传感器对象以及其pid对象
gyro = jy61p_i2c(I2C(1, scl=Pin(3), sda=Pin(2)))
gyro_pid = PID(5,5,2,0)

key = Pin(EXT_IRT_START, Pin.IN, Pin.PULL_UP)
def external_interrupt(key):
    # 消除抖动
    utime.sleep_ms(100)
    # 再次判断按键是否被按下
    if key.value() == 0:
        print('The button is pressed')
        return True
    return False



## 电机控制回调函数
def duty_set(timer):
    global freq_1, freq_2

    # 计算返回的频率
    freq_1 = counter1.freq
    freq_2 = counter2.freq

    # 对电机进行pid控制
    if target_1 == 0:
        m1.stop()
    elif target_1 > 0:
        m1.pid.set_target(target_1)
        duty_1 = m1.pid.compute(freq_1)
        m1.forward(duty_1)
    else:
        m1.pid.set_target(abs(target_1))
        duty_1 = m1.pid.compute(freq_1)
        m1.backward(duty_1)

    if target_2 == 0:
        m2.stop()
    elif target_2 > 0:
        m2.pid.set_target(target_2)
        duty_2 = m2.pid.compute(freq_2)
        m2.forward(duty_2)
    else:
        m2.pid.set_target(abs(target_2))
        duty_2 = m2.pid.compute(freq_2)
        m2.backward(duty_2)
        
    print('m1 freq=%.2f rpm=%.2f; m2 freq=%.2f rpm=%.2f' % (freq_1,freq_to_rpm(freq_1), freq_2,freq_to_rpm(freq_2)) )

# 计算目标转速对应的频率
def rpm_to_freq(rpm):
    return rpm * 3.095

def freq_to_rpm(freq):
    return freq / 3.095

# 初始化函数
def initialization():
    # 亮程序指示灯
    Pin(STATUS_LED, Pin.OUT).value(1)

    # 开始计算输入频率
    global start_time, timer_calc
    start_time = utime.ticks_us()
    counter1.start_count()
    counter2.start_count()

    # 开始duty set
    timer_dutyset.init(mode=Timer.PERIODIC, period=50, callback=duty_set)


# 电机设定函数
def motor_run(rpm1, rpm2):
    global target_1, target_2
    target_1 = rpm_to_freq(rpm1)
    target_2 = rpm_to_freq(rpm2)

# 电机转弯
def wheeling(direction, percent):
    global freq_1, freq_2
    bias = 50  # 调参
    val = bias * percent
    if direction == RIGHT:
        freq_1 = min(920, freq_1 + val/2)
        freq_2 = max(0, freq_2 - val/2)
    else:
        freq_1 = max(0, freq_1 - val/2)
        freq_2 = min(920, freq_2 + val/2)
        


dir_now = 0
dir_target = 0

# 陀螺仪转弯(particular angle)
def proceed_gyro(angle):
    global dir_now, dir_target
    dir_now = float(gyro.read_ang()[2])  #返回row轴角度
    dir_target = dir_now + angle
    # pid设置目标
    gyro_pid.set_target(dir_target)
    # 计算pid结果并归一
    control = to_percentage(gyro_pid.compute(float(gyro.read_ang()[2])))
    # 操作进行转弯
    wheeling(abs(angle)/angle, control)

# 化为-1～1之内的数
def to_percentage(x):
    return (2**x-2**(-x))/(2**x+2**(-x))

# 进程终止函数
def end_process():
    # 终止整个程序进程，不可恢复
    motor_run(0,0)
    m1.stop()
    m2.stop()
    timer_dutyset.deinit()
    counter1.end_count()
    counter2.end_count()
    Pin(STATUS_LED,Pin.OUT).value(0)
