import commands2


class Controller:
    def __init__(self, id):
        self.joystick = commands2.button.CommandXboxController(id)

        self.deadzone = 0.01

    def is_left_trigger_pressed(self):
        return self.joystick.getLeftTriggerAxis() > self.deadzone and self.joystick.getRightTriggerAxis() < self.deadzone
    
    def is_right_trigger_pressed(self):
        return self.joystick.getRightTriggerAxis() > self.deadzone
    
    def is_right_stick_moved(self):
        return abs(self.joystick.getRightY()) > self.deadzone
    
    def is_left_stick_moved(self):
        return abs(self.joystick.getLeftY()) > self.deadzone
    
    def get_left_stick_y(self):
        return -self.joystick.getLeftY()
    
    def get_right_stick_y(self):
        return -self.joystick.getRightY()
