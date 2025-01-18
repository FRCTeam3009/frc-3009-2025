import phoenix6.units
import commands2
import wpilib

class Elevator(object):

    def __init__(self):
        self.elevate = 0
        self.elevatorlength = 1

    def move_command(self, height: phoenix6.units.meters_per_second) -> commands2.Command:
        return MoveElevator(self, height)
    
    def change_height(self, speed):
        check = self.elevate + speed
        if check > self.elevatorlength:
            self.elevate = self.elevatorlength
        elif check < 0:
            self.elevate = 0
        elif self.elevate <= self.elevatorlength and self.elevate >= 0:
            self.elevate += speed
        

    def telemetry(self):
        box = wpilib.Mechanism2d(30, 30) # TODO these should be like the robot dimensions and the getRoot() part is relative to that.
        stationary_root = box.getRoot("Elevator", 15, 0.75)
        platform_root = box.getRoot("Platform", 15, self.elevate + 0.75)
        stationary_root.appendLigament(
            "stationary", 1, 90, 6, wpilib.Color8Bit(255, 255, 255)
        )
        root = platform_root.appendLigament(
            "platform", 1, 90
        )
        root.appendLigament(
            "CLAW", 1, 270, 6, wpilib.Color8Bit(0, 0, 255)
        )
        wpilib.SmartDashboard.putData("Elevator", box)
        
class MoveElevator(commands2.Command):
    def __init__(self, elevator: Elevator, speed: phoenix6.units.meters_per_second):
        self.elevator = elevator
        self.speed = speed
        
    def execute(self):
        self.elevator.change_height(self.speed)
