from time import sleep
from controllers import motor_controller, gyro_controller, ir_controller
from intersection_actions import IntersectionAction

NEW_LINE_TURNING_ANGLE = 38
OPPOSITE_DIRECTION_TURNING_ANGLE = 195

TURNING_ANGLE_THRESHOLD = 2

def map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

class LineFollower:
    def __init__(self):
        self.gyro_controller = gyro_controller.GyroController()
        self.motor_controller = motor_controller.MotorController()
        self.ir_controller = ir_controller.IRController()

        self.line_count = 0

        self.still_on_intersection = False
        self.last_action = ""

        self.intersection_list = []
        self.intersection_idx = 0

    def reset_actions(self):
        self.intersection_list = []
        self.intersection_idx = 0

    def is_line_detected(self):
        ir_values = self.ir_controller.get_ir_values()
        return (ir_values != 0 and ir_values != 7)

    def is_target_angle_reached(self, current_angle, target_angle, direction):
        if (direction == "r"):
            return (target_angle - TURNING_ANGLE_THRESHOLD) < current_angle < (target_angle + TURNING_ANGLE_THRESHOLD)
        else:
            return (target_angle + TURNING_ANGLE_THRESHOLD) > current_angle > (target_angle - TURNING_ANGLE_THRESHOLD)

    def rotate_to_match_target_angle(self, target_angle, direction):
        while (not self.is_target_angle_reached(self.gyro_controller.get_ang_z(), target_angle, direction)):
            if (direction == "l"):
                self.motor_controller.pivot_left()
            else:
                self.motor_controller.pivot_right()
            # print("current angle: %f" % gyro.get_ang_z())
            # print("target angle: %f" % target_angle)
        self.motor_controller.stop()
        # print("target_angle reached")
        # print("current angle z: %f" % self.gyro_controller.get_ang_z())

    def rotate_to_detect_line(self, direction):
        while (not self.is_line_detected()):
            if (direction == "l"):
                self.motor_controller.pivot_left()
            else:
                self.motor_controller.pivot_right()
        self.motor_controller.stop()

    def rotate(self, direction, turning_angle):
        # get the current z angle and map it to range(0, 360)
        angle_z = self.gyro_controller.get_ang_z()
        mapped_angle_z = map(angle_z, -180, 180, 0, 360)
        # print("current angle z: %f" % angle_z)
        # print("mapped current angle z: %f" % mapped_angle_z)

        # calculate the target angle and map it to range(-180, +180)
        mapped_target_angle = 0
        if (direction == "l"):
            mapped_target_angle = (mapped_angle_z + turning_angle) % 360
        else:
            mapped_target_angle = (mapped_angle_z - turning_angle) % 360

        target_angle = map(mapped_target_angle, 0, 360, -180, 180)
        # print("mapped target_angle: %f" % mapped_target_angle)
        # print("target_angle: %f" % target_angle)

        # rotate until target angle is reached
        # print("rotating until angle matched")
        self.rotate_to_match_target_angle(target_angle, direction)

        # at this point it is possible that the car is not on the line
        # so we make it rotate in the same direction as before until
        # the car is back on the line
        if (not self.is_line_detected()):
            # print("rotating until car on line")
            self.rotate_to_detect_line(direction)
            # print("car on line")
            # print("current angle z: %f" % self.gyro_controller.get_ang_z())

    def rotate_to_opposite_direction(self, direction):
        self.rotate(direction, OPPOSITE_DIRECTION_TURNING_ANGLE)

    def switch_to_new_lane(self, direction):
        self.rotate(direction, NEW_LINE_TURNING_ANGLE)

    def new_path(self, intersection_list: list[IntersectionAction]):
        self.do_follow_line(intersection_list=intersection_list)

    def continue_path(self):
        self.do_follow_line(intersection_list=None)

    def do_follow_line(self, intersection_list: list[IntersectionAction] = None):
        if intersection_list is not None:
            self.reset_actions()
            self.intersection_list = intersection_list

        print(self.intersection_list)
        print(self.intersection_idx)
        print(len(self.intersection_list) - 1)

        # while True:
        while self.intersection_idx < len(self.intersection_list):
            ir_values = self.ir_controller.get_ir_values()

            if (ir_values == 0): # 000
                self.motor_controller.stop()

                if (self.last_action == "slr"):
                    # print("overshoot")
                    # print("rotating until car on line")
                    self.rotate_to_detect_line("l")
                    # print("car on line")
                elif (self.last_action == "sll"):
                    # print("overshoot")
                    # print("rotating until car on line")
                    self.rotate_to_detect_line("r")
                    # print("car on line")
                elif (self.last_action == "ta"):
                    # print("overshoot")
                    # print("rotating until car on line")
                    self.rotate_to_detect_line("r")
                    # print("car on line")

                if (self.still_on_intersection):
                    self.still_on_intersection = False

            elif (ir_values == 2): # 010
                self.motor_controller.forward()
                if (self.still_on_intersection):
                    self.still_on_intersection = False

            elif (ir_values == 4 or ir_values == 6): # 100 | 110
                self.motor_controller.pivot_left()
                if (self.still_on_intersection):
                    self.still_on_intersection = False
            
            elif (ir_values == 1 or ir_values == 3): # 001 | 011
                self.motor_controller.pivot_right()
                if (self.still_on_intersection):
                    self.still_on_intersection = False
            
            elif (ir_values == 7): # 111
                if (self.still_on_intersection):
                    continue

                action = self.intersection_list[self.intersection_idx]
                print(self.intersection_idx)
                print(action)

                if (action == IntersectionAction.RIGHT):
                    print("switching to new lane - right")
                    self.switch_to_new_lane("r")
                    print("")
                    self.still_on_intersection = False
                    self.last_action = "slr"
                    self.intersection_idx += 1
                elif (action == IntersectionAction.LEFT):
                    print("switching to new lane - left")
                    self.switch_to_new_lane("l")
                    print("")
                    self.still_on_intersection = False
                    self.last_action = "sll"
                    self.intersection_idx += 1
                elif (action == IntersectionAction.IGNORE):
                    if (not self.still_on_intersection):
                        print("ignoring intersection \n")
                        self.motor_controller.forward()
                        self.still_on_intersection = True
                        self.last_action = ""
                        self.intersection_idx += 1
                elif (action == IntersectionAction.ROTATE_TO_OPPOSITE_DIRECTION):
                    print("rotating to opposite direction")
                    self.rotate_to_opposite_direction("l")
                    print("")
                    self.still_on_intersection = False
                    self.last_action = "sll"
                    self.intersection_idx += 2
                elif (action == IntersectionAction.STOP):
                    print("stop reached")
                    self.motor_controller.stop()
                    self.intersection_idx += 1
                    return
                    # sleep(5)
        
        self.motor_controller.stop()
        self.reset_actions()