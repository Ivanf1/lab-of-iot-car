import RPi.GPIO as gpio
from picar import back_wheels, front_wheels
import picar as picar
import mpu
import time

IR_LEFT_PIN = 20
IR_MIDDLE_PIN = 16
IR_RIGHT_PIN = 12 

def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

class LineFollower:
    def __init__(self):
        self.gyro = mpu.Mpu()
        self.gyro.base_initialize()
        self.gyro.calibrate()
        self.gyro.execute()

        picar.setup()
        db_file = "/home/ivan/programs/SunFounder_PiCar-V/remote_control/remote_control/driver/config"
        self.fw = front_wheels.Front_Wheels(debug=False, db=db_file)
        self.bw = back_wheels.Back_Wheels(debug=False, db=db_file)
        self.bw.ready()
        self.fw.ready()

        self.bw_speed = 30

        self.bw.speed = self.bw_speed

        gpio.setmode(gpio.BCM)

        gpio.setup(IR_LEFT_PIN, gpio.IN) # IR left
        gpio.setup(IR_MIDDLE_PIN, gpio.IN) # IR middle
        gpio.setup(IR_RIGHT_PIN, gpio.IN) # IR right

        self.line_count = 0

        self.still_on_intersection = False
        self.last_action = ""

    def get_ir_values(self):
        ir_values = (gpio.input(IR_LEFT_PIN) << 2) | (gpio.input(IR_MIDDLE_PIN) << 1) | (gpio.input(IR_RIGHT_PIN) << 0)
        return ir_values

    def is_line_detected(self):
        ir_values = self.get_ir_values()

        if (ir_values == 0 or ir_values == 7):
            return False
        return True

    def is_target_angle_reached(self, current_angle, target_angle, direction):
        if (direction == "r"):
            return (target_angle - 2) < current_angle < (target_angle + 2)
        else:
            return (target_angle + 2) > current_angle > (target_angle - 2)

    def rotate_to_match_target_angle(self, target_angle, direction):
        self.gyro.execute()
        while (not self.is_target_angle_reached(self.gyro.get_ang_z(), target_angle, direction)):
            if (direction == "l"):
                self.bw.pivot_left()
            else:
                self.bw.pivot_right()
            self.gyro.execute()
            # print("current angle: %f" % gyro.get_ang_z())
            # print("target angle: %f" % target_angle)
        self.bw.stop()
        print("target_angle reached")
        self.gyro.execute()
        print("current angle z: %f" % self.gyro.get_ang_z())

    def rotate_to_detect_line(self, direction):
        while (not self.is_line_detected()):
            if (direction == "l"):
                self.bw.pivot_left()
            else:
                self.bw.pivot_right()
        self.bw.stop()

    def switch_to_new_lane(self, direction):
        # update gyro values
        self.gyro.execute()
        # get the current z angle and map it to range(0, 360)
        angle_z = self.gyro.get_ang_z()
        mapped_angle_z = map(angle_z, -180, 180, 0, 360)
        print("current angle z: %f" % angle_z)
        print("mapped current angle z: %f" % mapped_angle_z)

        # calculate the target angle and map it to range(-180, +180)
        mapped_target_angle = 0
        if (direction == "l"):
            mapped_target_angle = (mapped_angle_z + 38) % 360
        else:
            mapped_target_angle = (mapped_angle_z - 38) % 360

        target_angle = map(mapped_target_angle, 0, 360, -180, 180)
        print("mapped target_angle: %f" % mapped_target_angle)
        print("target_angle: %f" % target_angle)

        # rotate until target angle is reached
        print("rotating until angle matched")
        self.rotate_to_match_target_angle(target_angle, direction)

        # at this point it is possible that the car is not on the line
        # so we make it rotate in the same direction as before until
        # the car is back on the line
        if (not self.is_line_detected()):
            print("rotating until car on line")
            self.rotate_to_detect_line(direction)
            print("car on line")
            self.gyro.execute()
            print("current angle z: %f" % self.gyro.get_ang_z())

        time.sleep(0.3)
        if (not self.is_line_detected()):
            print("overshoot")
            print("rotating until car on line")
            if (direction == "r"):
                self.rotate_to_detect_line("l")
            else:
                self.rotate_to_detect_line("r")
            print("car on line")
        
        print("lane switched")

    def do_follow_line(self):
        while True:
            ir_values = self.get_ir_values()

            if (ir_values == 0): # 000
                self.bw.stop()
                # recover from possible overshoot
                # if (self.last_action == "sll"):
                #     print("switch lane left overshoot")
                #     self.rotate_to_detect_line("r")
                # if (self.last_action == "slr"):
                #     print("switch lane right overshoot")
                #     self.rotate_to_detect_line("l")

                if (self.still_on_intersection):
                    self.still_on_intersection = False

            elif (ir_values == 2): # 010
                self.bw.speed = self.bw_speed
                self.bw.forward()
                if (self.still_on_intersection):
                    self.still_on_intersection = False

            elif (ir_values == 4 or ir_values == 6): # 100 | 110
                self.bw.speed = self.bw_speed
                self.bw.pivot_left()
                if (self.still_on_intersection):
                    self.still_on_intersection = False
            
            elif (ir_values == 1 or ir_values == 3): # 001 | 011
                self.bw.speed = self.bw_speed
                self.bw.pivot_right()
                if (self.still_on_intersection):
                    self.still_on_intersection = False
            
            elif (ir_values == 7): # 111
                if (not self.still_on_intersection):
                    self.line_count += 1
                if (self.line_count > 7):
                    self.bw.stop()

                if (self.line_count == 1):
                    print("switching to new lane - right")
                    self.switch_to_new_lane("r")
                    print("")
                    self.still_on_intersection = False
                    self.last_action = "slr"
                if (self.line_count == 2):
                    print("switching to new lane - left")
                    self.switch_to_new_lane("l")
                    print("")
                    self.still_on_intersection = False
                    self.last_action = "sll"
                if (self.line_count == 3):
                    if (not self.still_on_intersection):
                        print("ignoring intersection \n")
                        self.bw.speed = self.bw_speed
                        self.bw.forward()
                        self.still_on_intersection = True
                        self.last_action = ""
                if (self.line_count == 4):
                    print("switching to new lane - left")
                    self.switch_to_new_lane("l")
                    print("")
                    self.still_on_intersection = False
                    self.last_action = "sll"
                if (self.line_count == 5):
                    print("switching to new lane - left")
                    self.switch_to_new_lane("l")
                    print("")
                    self.still_on_intersection = False
                    self.last_action = "sll"
                if (self.line_count == 6):
                    print("switching to new lane - left")
                    self.switch_to_new_lane("l")
                    print("")
                    self.still_on_intersection = False
                    self.last_action = "sll"
                if (self.line_count == 7):
                    print("switching to new lane - right")
                    self.switch_to_new_lane("r")
                    print("")
                    self.still_on_intersection = False
                    self.last_action = "slr"