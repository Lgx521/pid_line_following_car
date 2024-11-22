from machine import Timer
from jy61p import jy61p_i2c
from motor_control import PID


class angle_loop:
    """
    attributions:

    timer;
    gyro;
    angle_now;
    angle_target;
    angle_now.

    read values:
    pid_compute_value
    """

    def __init__(self):
        self.angle_now = 0
        self.angle_target = 0
        self.angle_arr = [0,0,0]

        self.timer = Timer()
        self.gyro = jy61p_i2c()
        self.gyro.init()

        self.pid_compute_value = 0
        self.pid = PID(5,5,2)  # 这里需要调参

    def gyro_initialization(self):
        def timer_callback(timer):
            # 设置当前角度列表
            self.angle_arr = self.gyro.read_ang()
            self.angle_now = self.angle_arr[2]
            # 通过偏差值pid计算，返回归一化到-1～1的情况
            # 归一化以便于后面进行转弯的操作
            self.pid_compute_value = self.to_percentage(self.pid.compute(self.angle_now[2]))

        self.timer.init(mode=Timer.PERIODIC, period=50, callback=timer_callback)
       
    def set_angle_target(self, angle_target_delta):
        # angle_target_delta: 基于当前角度的增量值
        # Use after initialization
        self.angle_now = self.angle_arr[2]  # 这里直接读callback函数记录的值了
        self.angle_target = self.angle_now + angle_target_delta
        self.pid.set_target(self.angle_target)

    def to_percentage(x):
        return (2**x-2**(-x))/(2**x+2**(-x))
    