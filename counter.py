import utime
import functools
from machine import Pin, Timer

class Counter:

    def __init__(self, pin_num, interval=100):
        self.pin = Pin(pin_num,Pin.IN)
        self.interval = interval
        self.timer = Timer()
        self.timer.init(period=50, mode=Timer.PERIODIC, callback=self.calc_freq)
        self.time_start = utime.ticks_us()
        self.cycle = 0  # 指定时间内周期计数器
        self.freq = 0

    def count_pulse(self, p):
        self.cycle += 1


    # 定义回调函数，包含 self 和其他参数
    def calc_freq(self, timer):
        elapsed_time = utime.ticks_us() - self.time_start
        if elapsed_time > 0:
            self.freq = self.cycle / elapsed_time
            self.cycle = 0
            self.time_start = utime.ticks_us()
        

    # 启动定时器
    def start_count(self):
        # 计时器开始
        partial_callback = functools.partial(self.calc_freq)
        self.timer.init(mode=Timer.PERIODIC, period=self.interval, callback=partial_callback)
        # 开始计算频率
        self.time_start = utime.ticks_us()

        # 计数器开始
        partial_cntpulse = functools.partial(self.count_pulse)
        self.pin.irq(trigger=Pin.IRQ_RISING, handler=partial_cntpulse)


    def start_count2(self):
        # 计时器闭包
        def partial_callback(timer):
            elapsed_time = utime.ticks_us() - self.time_start
            if elapsed_time > 0:
                self.freq = self.cycle / elapsed_time
                self.cycle = 0
                self.time_start = utime.ticks_us()

        # 计数器闭包
        def partial_cntpulse(p):
            self.cycle += 1

        # 计时器开始
        self.timer.init(mode=Timer.PERIODIC, period=self.interval, callback=partial_callback)
        self.time_start = utime.ticks_us()

        # 计数器开始
        self.pin.irq(trigger=Pin.IRQ_RISING, handler=partial_cntpulse)
        
        

        


