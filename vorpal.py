from machine import Pin, I2C, PWM
import servo


class VorpalHexabot:

    SERVOMIN = 190
    SERVOMAX = 540

    NUM_LEGS = 6

    LEGS_ALL = 0b111111
    LEGS_LEFT = 0b111000
    LEGS_RIGHT = 0b000111
    LEGS_TRIPOD1 = 0b010101
    LEGS_TRIPOD2 = 0b101010
    LEGS_FRONT = 0b100001
    LEGS_MIDDLE = 0b010010
    LEGS_BACK = 0b001100
    LEGS_NONE = 0b0

    LEG_0 = 0b1
    LEG_1 = 0b10
    LEG_2 = 0b100
    LEG_3 = 0b1000
    LEG_4 = 0b10000
    LEG_5 = 0b100000

    KNEE_UP_MAX = 180
    KNEE_UP = 150
    KNEE_RELAX = 120
    KNEE_NEUTRAL = 90
    KNEE_CROUCH = 110
    KNEE_HALF_CROUCH = 80
    KNEE_STAND = 30
    KNEE_DOWN = 30
    KNEE_TIPTOES = 5
    KNEE_FOLD = 170
    KNEE_SCAMPER = KNEE_NEUTRAL-20
    KNEE_TRIPOD_UP = KNEE_NEUTRAL-40
    KNEE_TRIPOD_ADJ = 30

    HIP_NEUTRAL = 90
    HIP_SWING = 25
    HIP_SMALL_SWING = 10
    HIP_SWING_RIPPLE = 20
    HIP_FORWARD_MAX = 175
    HIP_FORWARD = HIP_NEUTRAL+HIP_SWING
    HIP_FORWARD_SMALL = HIP_NEUTRAL+HIP_SMALL_SWING
    HIP_BACKWARD = HIP_NEUTRAL-HIP_SWING
    HIP_BACKWARD_SMALL = HIP_NEUTRAL-HIP_SMALL_SWING
    HIP_BACKWARD_MAX = 0
    HIP_FORWARD_RIPPLE = HIP_NEUTRAL+HIP_SWING_RIPPLE
    HIP_BACKWARD_RIPPLE = HIP_NEUTRAL+HIP_SWING_RIPPLE
    HIP_FOLD = 150

    NOMOVE = -1
    LEFT_START = 3
    RIGHT_START = 0
    KNEE_OFFSET = 6

    CYCLE_TIME_TRIPOD = 750
    CYCLE_TIME_RIPPLE = 1800
    CYCLE_TIME_FIGHT = 660

    BATTERYSAVER = 5000

    def __init__(self, i2c, beeper_pin='P11'):
        self.i2c = i2c
        self.servos = servo.Servos(i2c)
        self.init_beeper(beeper_pin)

    def exec_cmd(self, cmd):
        print("exec {}".format(cmd))

    def set_hip_raw(self, leg, pos):
        self.servos.position(leg, degrees=pos)

    def set_hip(self, leg, pos):
        if leg >= self.LEFT_START:
            pos = 180 - pos
        self.set_hip_raw(leg, pos)

    def set_hip_adj(self, leg, pos, adj):
        if self.is_front_leg(leg):
            pos -= adj
        elif self.is_back_leg(leg):
            pos += adj
        # reverse left for consistent forward motion
        if leg > self.LEFT_START:
            pos = 180 - pos

        self.set_hip_raw(leg, pos)

    def set_knee(self, leg, pos):
        # find the associated with leg if this is not already a knee
        if leg < self.KNEE_OFFSET:
            leg += self.KNEE_OFFSET
        self.servos.position(leg, degrees=pos)

    def set_leg_raw(self, legmask, hip_pos, knee_pos, adj, raw):
        for idx in range(self.NUM_LEGS):
            if legmask & 0b1:
                if hip_pos != self.NOMOVE:
                    if raw == 0:
                        #print("set hip {}".format(hip_pos))
                        self.set_hip_adj(idx, hip_pos, adj)
                    else:
                        #print("set hip raw {}".format(hip_pos))
                        self.set_hip_raw(idx, hip_pos)
                if knee_pos != self.NOMOVE:
                    # set_knee
                    #print("set knee position {}".format(knee_pos))
                    self.set_knee(idx, knee_pos)
            legmask = legmask >> 1

    def set_leg(self, legmask, hip_pos, knee_pos, adj):
        self.set_leg_raw(legmask, hip_pos, knee_pos, adj, 0)

    def beep(self):
        print("beep")
        self.beeper_ch.duty_cycle(0.9)
        time.sleep_ms(80)
        self.beeper_ch.duty_cycle(0.0)

    def init_beeper(self, pin):
        self.beeper_pin = Pin(pin)
        self.beeper_pwm = PWM(0, frequency=1000)
        self.beeper_ch = self.beeper_pwm.channel(0, pin=self.beeper_pin, duty_cycle=0.5)
        self.beeper_ch.duty_cycle(0.0)

    def is_front_leg(self, leg):
        return leg==0 or leg==5

    def is_middle_leg(self, leg):
        return leg==1 or leg==4

    def is_back_leg(self, leg):
        return leg==2 or leg==3

    def is_left_legt(self, leg):
        return leg==0 or leg==1 or leg==2

    def is_right_leg(self, leg):
        return leg==3 or leg==4 or leg==5

    def turn(self, ccw, hipforward, hipbackward, kneeup, kneedown, timeperiod):
        if ccw:
            tmp = hipforward
            hipforward = hipbackward
            hipbackward = tmp

        NUM_TURN_PHASES = 6
        FBSHIFT_TURN = 40

        t = (time.time() * 1000) % timeperiod
        phase = (NUM_TURN_PHASES*t)/timeperiod

        if phase == 0:
            # in this phase, center-left and noncenter-right legs raise up at
            # the knee
            self.set_leg(self.LEGS_TRIPOD1, self.NOMOVE, kneeup, 0)
        elif phase == 1:
            # in this phase, center-left and noncenter-right legs raise up at
            # the knee put the first set of legs back down on the ground
            self.set_leg(self.LEGS_TRIPOD1, hipforward, self.NOMOVE, FBSHIFT_TURN, 1)
            self.set_leg(self.LEGS_TRIPOD2, hipbackward, self.NOMOVE, FBSHIFT_TURN, 1)
        elif phase == 2:
            # now put the first set of legs back down on the ground
            self.set_leg(self.LEGS_TRIPOD1, self.NOMOVE, kneedown, 0)
        elif phase == 3:
            # lift up to the other set of legs at the knee
            self.set_leg(self.LEGS_TRIPOD2, self.NOMOVE, kneeup, 0)
        elif phase == 4:
            # similar to phase 1, move raised legs CW and lowered legs CCW
            self.set_leg_raw(self.LEGS_TRIPOD1, hipbackward, self.NOMOVE, FBSHIFT_TURN, 1)
            self.set_leg_raw(self.LEGS_TRIPOD2, hipforward, self.NOMOVE, FBSHIFT_TURN, 1)
        elif phase == 5:
            # put the second set of legs down and cycle repeats
            self.set_leg(self.LEGS_TRIPOD2, self.NOMOVE, kneedown, 0)

    def stand(self):
        self.set_leg(self.LEGS_ALL, self.HIP_NEUTRAL, self.KNEE_STAND, 0)

    def stand_90_degrees(self):
        # used to install servos, sets all servos to 90 deg
        self.set_leg(self.LEGS_ALL, 90, 90, 0)

    def laydown(self):
        self.set_leg(self.LEGS_ALL, self.HIP_NEUTRAL, self.KNEE_UP, 0)

    def tiptoes(self):
        self.set_leg(self.LEGS_ALL, self.HIP_NEUTRAL, self.KNEE_TIPTOES, 0)

    def turn_deg(self, degrees):
        # turn tripod 1 up
        self.set_leg(self.LEGS_TRIPOD1, self.NOMOVE, self.KNEE_UP, 0)
        # move hip with standing tripod
        time.sleep_ms(250)
        self.set_leg_raw(self.LEGS_TRIPOD2, self.HIP_NEUTRAL-degrees, self.NOMOVE, 0, 1)
        # turn tripod 1 down
        time.sleep_ms(250)
        self.set_leg(self.LEGS_TRIPOD1, self.NOMOVE, self.KNEE_STAND, 0)
        # turn tripod 2 up
        time.sleep_ms(250)
        self.set_leg(self.LEGS_TRIPOD2, self.NOMOVE, self.KNEE_UP, 0)
        # move tripod 2 hip back to neutral
        time.sleep_ms(250)
        self.set_leg(self.LEGS_TRIPOD2, self.HIP_NEUTRAL, self.KNEE_UP, 0)
        # turn tripod 2 down
        time.sleep_ms(250)
        self.set_leg(self.LEGS_TRIPOD2, self.HIP_NEUTRAL, self.KNEE_STAND, 0)

    def standby(self):
        for i in range(self.NUM_LEGS):
            self.servos.release(i)


from machine import I2C
i2c = I2C(baudrate=400000)
robot = VorpalHexabot(i2c)
i = 0
robot.stand()
time.sleep(1)
# if i % 3 == 0:
#     time.sleep_ms(2000)
# robot.set_leg(VorpalHexabot.LEG_5, VorpalHexabot.HIP_NEUTRAL, VorpalHexabot.KNEE_UP_MAX, 0)
# robot.beep()
# time.sleep_ms(200)
# robot.set_leg(VorpalHexabot.LEG_5, VorpalHexabot.HIP_NEUTRAL, VorpalHexabot.KNEE_NEUTRAL, 0)
# time.sleep_ms(200)
# i += 1
#robot.tiptoes()
#robot.laydown()

while True:
    robot.turn_deg(45)
    time.sleep(2)
