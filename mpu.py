import smbus
import time
import math

class Mpu:
        Device_Address = 0x68   # MPU6050 device address

        #some MPU6050 Registers and their Address
        PWR_MGMT_1   = 0x6B
        SMPLRT_DIV   = 0x19
        CONFIG       = 0x1A
        GYRO_CONFIG  = 0x1B
        ACCEL_CONFIG = 0x1C
        INT_ENABLE   = 0x38
        ACCEL_XOUT_H = 0x3B
        ACCEL_YOUT_H = 0x3D
        ACCEL_ZOUT_H = 0x3F
        GYRO_XOUT_H  = 0x43
        GYRO_YOUT_H  = 0x45
        GYRO_ZOUT_H  = 0x47

        DEFAULT_ACCEL_COEFF = 0.02
        DEFAULT_GYRO_COEFF = 0.98

        # For calibration
        DISCARDED_MEASURES = 100
        CALIBRATION_MEASURES = 5000 #5000
        CHECKING_MEASURES = 50
        ACCEL_PREOFFSET_MAGIC_NUMBER = 8
        GYRO_PREOFFSET_MAGIC_NUMBER = 4

        ACCEL_TRANSFORMATION_NUMBER = 0.00006103515 # (1 / 16384) precalculated
        GYRO_TRANSFORMATION_NUMBER = 0.01525878906 # (1 / 65.536) precalculated

        RAD_TO_DEG = 57.295779513082320876798154814105

        filterAccelCoeff = DEFAULT_ACCEL_COEFF
        filterGyroCoeff = DEFAULT_GYRO_COEFF

        rawAccX = 0
        rawAccY = 0
        rawAccZ = 0
        rawGyroX = 0
        rawGyroY = 0
        rawGyroZ = 0

        accX = 0
        accY = 0
        accZ = 0
        gyroX = 0
        gyroY = 0
        gyroZ = 0

        angX = 0
        angY = 0
        angZ = 0

        angAccX = 0
        angAccY = 0

        angGyroX = 0
        angGyroY = 0
        angGyroZ = 0

        intervalStart = 0
        dt = 0

        bus = smbus.SMBus(1)

        def millis(self):
                return round(time.time() * 1000)

        def wrap(self, angle):
                while (angle > +180):
                        angle -= 360
                while (angle < -180):
                        angle += 360
                return angle

        def angle_average(self, wa, a, wb, b):
                return self.wrap(wa * a + wb * (a + self.wrap(b - a)))

        def base_initialize(self):
                self.bus.write_byte_data(self.Device_Address, self.SMPLRT_DIV, 0x00)
                self.bus.write_byte_data(self.Device_Address, self.CONFIG, 0x00)
                self.bus.write_byte_data(self.Device_Address, self.GYRO_CONFIG, 0x08)
                self.bus.write_byte_data(self.Device_Address, self.ACCEL_CONFIG, 0x00)
                self.bus.write_byte_data(self.Device_Address, self.PWR_MGMT_1, 0x01)

                self.angX = 0
                self.angY = 0
                self.angZ = 0

                self.intervalStart = self.millis()

        def execute(self):
                # Updating raw data before processing it
                self.update_raw_accel()
                self.update_raw_gyro()

                # Computing readable accel/gyro data
                self.accX = self.rawAccX * self.ACCEL_TRANSFORMATION_NUMBER
                self.accY = self.rawAccY * self.ACCEL_TRANSFORMATION_NUMBER
                self.accZ = self.rawAccZ * self.ACCEL_TRANSFORMATION_NUMBER
                self.gyroX = (self.rawGyroX - self.gyroXOffset) * self.GYRO_TRANSFORMATION_NUMBER
                self.gyroY = (self.rawGyroY - self.gyroYOffset) * self.GYRO_TRANSFORMATION_NUMBER
                self.gyroZ = (self.rawGyroZ - self.gyroZOffset) * self.GYRO_TRANSFORMATION_NUMBER

                # Computing accel angles
                self.angAccX = self.wrap((math.atan2(self.accY, math.sqrt(self.accZ * self.accZ + self.accX * self.accX))) * self.RAD_TO_DEG)
                self.angAccY = self.wrap((math.atan2(self.accX, math.sqrt(self.accZ * self.accZ + self.accY * self.accY))) * self.RAD_TO_DEG)

                # Computing gyro angles
                self.dt = (self.millis() - self.intervalStart) * 0.001
                self.angGyroX = self.wrap(self.angGyroX + self.gyroX * self.dt)
                self.angGyroY = self.wrap(self.angGyroY + self.gyroY * self.dt)
                self.angGyroZ = self.wrap(self.angGyroZ + self.gyroZ * self.dt)

                # Computing complementary filter angles
                self.angX = self.angle_average(self.filterAccelCoeff, self.angAccX, self.filterGyroCoeff, self.angX + self.gyroX * self.dt)
                self.angY = self.angle_average(self.filterAccelCoeff, self.angAccY, self.filterGyroCoeff, self.angY + self.gyroY * self.dt)
                self.angZ = self.angGyroZ

                # Resetting the integration timer
                self.intervalStart = self.millis()

        def update_raw_accel(self):
                # Read Accelerometer raw value
                self.rawAccX = self.read_raw_data(self.ACCEL_XOUT_H)
                self.rawAccY = self.read_raw_data(self.ACCEL_YOUT_H)
                self.rawAccZ = self.read_raw_data(self.ACCEL_ZOUT_H)
                
        def update_raw_gyro(self):
                # Read Gyroscope raw value
                self.rawGyroX = self.read_raw_data(self.GYRO_XOUT_H)
                self.rawGyroY  = self.read_raw_data(self.GYRO_YOUT_H)
                self.rawGyroZ = self.read_raw_data(self.GYRO_ZOUT_H)

        def calibrate(self):
                print("start calibration")
                for i in range(self.DISCARDED_MEASURES):
                        self.update_raw_accel()
                        self.update_raw_gyro()
                        time.sleep(0.02)
                print("calibration first step completed")

                sumGyroX = 0 
                sumGyroY = 0 
                sumGyroZ = 0 

                for j in range(self.CALIBRATION_MEASURES):
                        self.update_raw_accel()
                        self.update_raw_gyro()
                        sumGyroX += self.get_raw_gyro_x()
                        sumGyroY += self.get_raw_gyro_y()
                        sumGyroZ += self.get_raw_gyro_z()
                        time.sleep(0.02)
                print("calibration second step completed")
                
                sumGyroX /= self.CALIBRATION_MEASURES
                sumGyroY /= self.CALIBRATION_MEASURES
                sumGyroZ /= self.CALIBRATION_MEASURES

                self.set_gyro_offsets(sumGyroX, sumGyroY, sumGyroZ)
                print("calibration completed")

        def read_raw_data(self, addr):
                # Accelero and Gyro value are 16-bit
                high = self.bus.read_byte_data(self.Device_Address, addr)
                low = self.bus.read_byte_data(self.Device_Address, addr+1)
            
                #concatenate higher and lower value
                value = ((high << 8) | low)
                
                #to get signed value from mpu6050
                if(value > 32768):
                        value = value - 65536
                return value

        def get_gyro_x_offset(self):
                return self.gyroXOffset

        def get_gyro_y_offset(self):
                return self.gyroYOffset

        def get_gyro_z_offset(self):
                return self.gyroZOffset

        def set_gyro_offsets(self, x, y, z):
                self.gyroXOffset = x
                self.gyroYOffset = y
                self.gyroZOffset = z

        def get_raw_acc_x(self):
                return self.rawAccX

        def get_raw_acc_y(self):
                return self.rawAccY
        
        def get_raw_acc_z(self):
                return self.rawAccZ

        def get_raw_gyro_x(self):
                return self.rawGyroX

        def get_raw_gyro_y(self):
                return self.rawGyroY

        def get_raw_gyro_z(self):
                return self.rawGyroZ

        def get_acc_x(self):
                return self.accX

        def get_acc_y(self):
                return self.accY
        
        def get_acc_z(self):
                return self.accZ

        def get_gyro_x(self):
                return self.gyroX

        def get_gyro_y(self):
                return self.gyroY

        def get_gyro_z(self):
                return self.gyroZ

        def get_ang_x(self):
                return self.angX

        def get_ang_y(self):
                return self.angY

        def get_ang_z(self):
                return self.angZ

        def get_filter_acc_coeff(self):
                return self.filterAccelCoeff

        def get_filter_gyro_coeff(self):
                return self.filterGyroCoeff

        def set_filter_acc_coeff(self, coeff):
                self.filterAccelCoeff = coeff

        def set_filter_gyro_coeff(self, coeff):
                self.filterGyroCoeff = coeff
