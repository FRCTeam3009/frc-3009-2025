import phoenix6.units
import commands2
import wpilib
import phoenix6.hardware
import time
import wpilib.simulation
import typing
import ntcore
import wpimath.units
import subsystems.wrist
from generated.tuner_constants import TunerConstants
import wpimath.controller

SPEED = 0.3

class Elevator(commands2.Subsystem):

    def __init__(self):
        commands2.CommandScheduler.registerSubsystem(self)
        self.main_motor = phoenix6.hardware.TalonFX(TunerConstants.elevator_main_id, "rio")
        self.follower_motor = phoenix6.hardware.TalonFX(TunerConstants.elevator_follower_id, "rio")
        self.follower_motor.set_control(phoenix6.controls.follower.Follower(TunerConstants.elevator_main_id, False))

        self.start_position = self.main_motor.get_position().value_as_double - 1
        self.up_limit = self.start_position + MoveElevatorToPosition.L4 # NOTE motor moves backwards to "up" limit is negative.
        self.lower_limit = MoveElevatorToPosition.lower_limit
        self.prev_time = time.time()

        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self.nt_table = self.nt_instance.getTable("elevator")

        self.elevator_topic = self.nt_table.getDoubleTopic("elevator_position")
        self.elevator_publish = self.elevator_topic.publish()
        self.elevator_publish.set(0.0)

        self.position_to_hold = 0

    def sim_update(self):
        self.prev_time = time.time()
    
    def change_height(self, speed: float):
        speed *= -1
        self.main_motor.set(speed)

        if self.get_position() <= self.up_limit and speed < 0:
            # Stop the motor if we went too high
            self.main_motor.set(0)
        elif self.get_position() >= self.lower_limit and speed > 0:
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

    def telemetry(self, wrist: subsystems.wrist.Wrist):
        box = wpilib.Mechanism2d(29.5, 29.5)
        stationary_root = box.getRoot("Elevator", 15, 0.75)

        elevator_position = -1 * self.get_position() * 30/1000.0
        wrist_rotation = wrist.coral_wrist_sim.getAbsoluteEncoderSim().getPosition()

        platform_root = box.getRoot("Platform", 15, elevator_position)
        stationary_root.appendLigament(
            "stationary", 1, 90, 6, wpilib.Color8Bit(255, 255, 255)
        )
        root = platform_root.appendLigament(
            "platform", 1, 90
        )
        root.appendLigament(
           "wrist", 1, 180 + wrist_rotation, 6, wpilib.Color8Bit(0, 0, 255)
        )
        wpilib.SmartDashboard.putData("Elevator", box)

        self.elevator_publish.set(self.get_position())
        
    
    def get_position(self) -> float:
        return self.main_motor.get_position().value_as_double
    

    

class MoveElevatorCommand(commands2.Command):
    def __init__(self, elevator: Elevator, amount: typing.Callable[[], float]):
        self.elevator = elevator
        self.amount = amount
    
    def execute(self):
        self.elevator.position_to_hold += self.amount()

class MoveElevatorToPosition(commands2.Command):
    # NOTE Moving the elevator up is actually negative values.
    L4 = -87.5
    L3 = -49
    L2 = -16
    L1 = 1.0
    pickup = 1.0
    lower_limit = 6.2
    auto_pose = -10
    
    def __init__(self, elevator: Elevator, position : float):
        self.elevator = elevator
        self.position = position

    def execute(self):
        self.elevator.position_to_hold = self.position

    def isFinished(self):
        return True

class HoldPositionCommand(commands2.Command):
    def __init__(self, elevator: Elevator):
        self.elevator = elevator
        self.addRequirements(self.elevator)

        self.elevatorPID = wpimath.controller.PIDController(0, 0, 0)
        self.elevatorFeedForward = wpimath.controller.ElevatorFeedforward(0, 0, 0)
    
    def execute(self):
        feedback = self.elevatorPID.calculate(self.elevator.get_position(), self.elevator.position_to_hold)
        feedforward = self.elevatorFeedForward.calculate(self.elevator.main_motor.get_velocity().value_as_double)
        self.elevator.main_motor.setVoltage(feedback + feedforward)