import RPi.GPIO as gpio

IR_LEFT_PIN = 20
IR_MIDDLE_PIN = 16
IR_RIGHT_PIN = 12 

class IRController:
    def __init__(self) -> None:
        gpio.setmode(gpio.BCM)

        gpio.setup(IR_LEFT_PIN, gpio.IN)
        gpio.setup(IR_MIDDLE_PIN, gpio.IN)
        gpio.setup(IR_RIGHT_PIN, gpio.IN)

    def get_ir_values(self):
        ir_values = (gpio.input(IR_LEFT_PIN) << 2) | (gpio.input(IR_MIDDLE_PIN) << 1) | (gpio.input(IR_RIGHT_PIN) << 0)
        return ir_values
    


