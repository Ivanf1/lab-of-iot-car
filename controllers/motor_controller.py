from picar import back_wheels 
import picar as picar

DEFAULT_SPEED = 30
PIVOT_SPEED = 20

class MotorController:
    def __init__(self) -> None:
        picar.setup()
        db_file = "/home/ivan/programs/SunFounder_PiCar-V/remote_control/remote_control/driver/config"
        self.bw = back_wheels.Back_Wheels(debug=False, db=db_file)
        self.bw.ready()

    def forward(self):
        self.bw.speed = DEFAULT_SPEED
        self.bw.forward()

    def pivot_left(self):
        self.bw.speed = PIVOT_SPEED
        self.bw.pivot_left()

    def pivot_right(self):
        self.bw.speed = PIVOT_SPEED
        self.bw.pivot_right()

    def stop(self):
        self.bw.stop()