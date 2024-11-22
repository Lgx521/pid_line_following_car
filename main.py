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
M3 = 12
M4 = 11
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
gyro = jy61p_i2c()
# gyro_pid = PID(3,0,25,0)
gyro_pid = PID(3,0,25,0)

key = Pin(EXT_IRT_START, Pin.IN, Pin.PULL_UP)
def external_interrupt(key):
    # 消除抖动
    utime.sleep_ms(150)
    # 再次判断按键是否被按下
    if key.value() == 0:
        print('The button is pressed')
        return True
    return False



## 电机控制回调函数
def duty_set(timer):
    global freq_1, freq_2, target_1, target_2

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
        
    print('m1 freq=%.2f rpm=%.2f;  m2 freq=%.2f rpm=%.2f' % (freq_1,freq_to_rpm(freq_1), freq_2,freq_to_rpm(freq_2)) )
    print('m1 target=%.2f, m2 target=%.2f'%(target_1, target_2))


# 计算目标转速对应的频率
def rpm_to_freq(rpm):
    return rpm * 3.095

def freq_to_rpm(freq):
    return freq / 3.095

# 初始化函数
def initialization():
    # 亮程序指示灯
    Pin(STATUS_LED, Pin.OUT).value(1)
    
    gyro.init()

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
def wheeling(percent):
    global target_1, target_2
    bias = 1.2  # 调参 1.0 正常转弯，超调较小
    val = bias * percent
    target_1 = target_1 + val/2
    target_2 = target_2 - val/2
    

def angle_difference(angle1, angle2):

    # 确保角度在 0～360 范围内
    angle1 = angle1 % 360
    angle2 = angle2 % 360

    # 计算直接差值
    diff = angle2 - angle1

    # 确保差值在 -180 到 180 范围
    if diff > 180:
        diff -= 360
    elif diff < -180:
        diff += 360

    return diff
        

gyro_status = 0
dir_target = 0
dir_inprogress=0

def gyro_callback(t):
    global dir_target, dir_inprogress
    
    dir_inprogress=float(gyro.read_ang()[2])
    
    error=angle_difference(dir_target,dir_inprogress)
    
    if abs(error) < 1:
        print('control=OFF, tar=%.2f, dirnow=%.2f'%(dir_target, dir_inprogress))
        global target_1, target_2
        mean = (target_1 + target_2) / 2
        target_1 = mean
        target_2 = mean
        return
    
    control = gyro_pid.compute_2(error) / 40
    
    if control > 30:
        control=30
    elif control < -30:
        control = -30
    print('control=%.2f, tar=%.2f, dirnow=%.2f'%(control, dir_target, dir_inprogress))
    # 操作进行转弯
    if control!=0:
        wheeling(control)
        
gyro_timer = Timer()


# 陀螺仪转弯(particular angle)
def proceed_gyro(angle):
    '''
    angle 为一个增量，范围为-180～180，负号表示右转，正号左转
    '''
    global gyro_status, dir_target
    
    
    # 设定目标方向
    dir_now = float(gyro.read_ang()[2])
    
    dir_target = dir_now + angle
    if dir_target > 180:
        dir_target -= 360
    elif dir_target < -180:
        dir_target += 360
        
    # pid设置目标
    gyro_pid.set_target(dir_target)
    # 开始计时
    if gyro_status == 0:
        gyro_timer.init(mode=Timer.PERIODIC, period=50, callback=gyro_callback)

    gyro_status = 1 

    




# 进程终止函数
def end_process():
    # 终止整个程序进程，不可恢复
    motor_run(0,0)
    m1.stop()
    m2.stop()
    timer_dutyset.deinit()
    gyro_timer.deinit()
    counter1.end_count()
    counter2.end_count()
    Pin(STATUS_LED,Pin.OUT).value(0)
    
    
if __name__ == '__main__':
    
    initialization()
    
    utime.sleep(1) # 等待接线
    
    motor_run(150,150)
    
    proceed_gyro(0)
    
    utime.sleep(3)
    
    proceed_gyro(-90)
    
    utime.sleep(8)
    

    Pin(17,Pin.OUT).value(0)   
    end_process()

