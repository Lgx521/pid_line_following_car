from machine import PWM, Pin, Timer
import utime


class motor:
    
    def __init__(self, pin_1=7, pin_2=8, pin_pwm=6, pin_pwm_in=21):
        
        self.pin_1 = pin_1
        self.pin_2 = pin_2
        self.pin_pwm = pin_pwm
        self.pin_pwm_in = pin_pwm_in
        
        self.pin1 = Pin(pin_1,Pin.OUT)
        self.pin2 = Pin(pin_2,Pin.OUT)
                
        self.pwm = PWM(Pin(pin_pwm,Pin.OUT), freq = 1000)

        self.Kp = 30
        self.Ki = 100
        self.Kd = 5

        self.pid = PID(self.Kp, self.Ki, self.Kd, 0)
                
        # stop the motor
        self.pin1.value(0)
        self.pin2.value(0)
    
    def forward(self, duty):
        self.pwm.init()
        self.pwm.duty_u16(int(max(0,min(65535,duty))))  # 避免超出调节区间
        self.pin1.value(1)
        self.pin2.value(0)
        
    def backward(self, duty):
        self.pwm.init()
        self.pwm.duty_u16(int(max(0,min(65535,duty))))  # 避免超出调节区间
        self.pin1.value(0)
        self.pin2.value(1)
        
    def stop(self):
        self.pin1.value(0)
        self.pin2.value(0)
        self.pwm.deinit()




class PID:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

    def set_target(self,target):
        self.setpoint = target

    def set_param(self,kp,ki,kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def compute(self, measurement):
        error = self.setpoint - measurement
#         print(error)
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative    



class counter:
    def __init__(self, pin_num, interval=50):
        self.pin = Pin(pin_num,Pin.IN)
        self.interval = interval
        self.timer = Timer()
        self.time_start = utime.ticks_us()
        self.cycle = 0  # 指定时间内周期计数器
        self.freq = 0


    def start_count(self):
        # 计时器闭包
        def partial_callback(timer):
            elapsed_time = utime.ticks_us() - self.time_start
            if elapsed_time > 0:
                self.freq = self.cycle * 1000000 / elapsed_time
                self.cycle = 0
                self.time_start = utime.ticks_us()

        # 计数器闭包
        def partial_cntpulse(p):
            self.cycle += 1

        # 计时器开始
        self.time_start = utime.ticks_us()
        self.timer.init(mode=Timer.PERIODIC, period=self.interval, callback=partial_callback)

        # 计数器开始
        self.pin.irq(trigger=Pin.IRQ_RISING, handler=partial_cntpulse)

    # 终止计时器
    def end_count(self):
        self.timer.deinit()


