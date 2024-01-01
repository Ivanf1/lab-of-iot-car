import mpu

class GyroController:
    def __init__(self) -> None:
        self.gyro = mpu.Mpu()
        self.gyro.base_initialize()
        self.gyro.set_calibration_measures(1500)
        self.gyro.calibrate()
        self.gyro.execute()

    def get_ang_z(self):
        self.gyro.execute()
        return self.gyro.get_ang_z()

