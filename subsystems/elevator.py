import phoenix6.units
import commands2
import wpilib
import phoenix6.hardware

class Elevator(object):

    def __init__(self):
        self.elevate = 0
        self.elevator_length = 1
        self.left_motor = phoenix6.hardware.TalonFX(31, "rio") #31 is device id
        self.right_motor = phoenix6.hardware.TalonFX(32, "rio") #31 is device id

    def move_command(self, height: phoenix6.units.meters_per_second) -> commands2.Command:
        return MoveElevatorCommand(self, height)
    
    def change_height(self, speed):
        check = self.elevate + speed
        if check > self.elevator_length:
            self.elevate = self.elevator_length
        elif check < 0:
            self.elevate = 0
            self.right_motor.set(0)
            self.left_motor.set(0)
        elif self.elevate <= self.elevator_length and self.elevate >= 0:
            self.elevate += speed
            self.right_motor.set(speed)
            self.left_motor.set(speed)
        

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
        
class MoveElevatorCommand(commands2.Command):
    def __init__(self, elevator: Elevator, speed: phoenix6.units.meters_per_second):
        self.elevator = elevator
        self.speed = speed
        
    def execute(self):
        self.elevator.change_height(self.speed)
