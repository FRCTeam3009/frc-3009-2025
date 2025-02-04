import phoenix6.units
import commands2
import wpilib
import phoenix6.hardware
import time
from generated.tuner_constants import TunerConstants

class Elevator(object):

    def __init__(self):
        self.main_motor = phoenix6.hardware.TalonFX(TunerConstants._elevator_main_id, "rio")
        self.follower_motor = phoenix6.hardware.TalonFX(TunerConstants._elevator_follower_id, "rio")

        self.follower_motor.set_control(phoenix6.controls.follower.Follower(TunerConstants._elevator_main_id, False))

        self.start_position = self.main_motor.get_position().value_as_double
        self.up_limit = 100.0
        self.prev_time = time.time()

    def move_command(self, speed: float) -> commands2.Command:
        return MoveElevatorCommand(self, speed)
    
    def change_height(self, speed: float):
        if self.get_position() >= self.up_limit and speed > 0:
            # Stop the motor if we went too high
            self.main_motor.set(0)
        elif self.get_position() <= self.start_position and speed < 0:
            # Stop the motor if we went too low
            self.main_motor.set(0)
        else:
            # Move motor
            self.main_motor.set(speed)

            # Update sim state
            difference = time.time() - self.prev_time
            rpm = 6380
            seconds_per_minute = 60
            rotations = rpm / seconds_per_minute * difference * speed
            rotations += self.get_position()
            self.main_motor.sim_state.set_raw_rotor_position(rotations)
        
        self.prev_time = time.time()

        

    def telemetry(self):
        box = wpilib.Mechanism2d(30, 30) # TODO these should be like the robot dimensions and the getRoot() part is relative to that.
        stationary_root = box.getRoot("Elevator", 15, 0.75)
        platform_root = box.getRoot("Platform", 15, self.get_position())
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
        
    
    def get_position(self) ->float :
        return self.main_motor.get_position().value_as_double
    

class MoveElevatorCommand(commands2.Command):
    def __init__(self, elevator: Elevator, speed: float):
        self.elevator = elevator
        self.speed = speed
        
    def execute(self):
        self.elevator.change_height(self.speed)
