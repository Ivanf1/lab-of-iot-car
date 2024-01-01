from picar import back_wheels, front_wheels
import picar as picar
import mpu

DEFAULT_SPEED = 30

class CarController:
    def __init__(self) -> None:
        self.gyro = mpu.Mpu()
        self.gyro.base_initialize()
        self.gyro.set_calibration_measures(1500)
        self.gyro.calibrate()
        self.gyro.execute()

        picar.setup()
        db_file = "/home/ivan/programs/SunFounder_PiCar-V/remote_control/remote_control/driver/config"
        self.fw = front_wheels.Front_Wheels(debug=False, db=db_file)
        self.bw = back_wheels.Back_Wheels(debug=False, db=db_file)
        self.bw.ready()
        self.fw.ready()

        self.bw.speed = DEFAULT_SPEED

    def forward(self):
        self.bw.speed = DEFAULT_SPEED
        self.bw.forward()

    def pivot_left(self):
        self.bw.speed = DEFAULT_SPEED
        self.bw.pivot_left()

    def pivot_right(self):
        self.bw.speed = DEFAULT_SPEED
        self.bw.pivot_right()

    def stop(self):
        self.bw.stop()





