import phoenix6.units
import commands2
import wpilib
import wpimath
import phoenix6.hardware
import phoenix5
import time
import rev
import wpilib.simulation
import wpimath.system
import wpimath.system.plant
import typing
import ntcore
from generated.tuner_constants import TunerConstants

# TODO we'll want to test the april tag odometry to see if we need to customize it at all
# (e.g. does it think we teleport around the field or does it update pretty smoothly.)

# TODO test the auto modes on a full size field.

# TODO errors with the robot relative drive if there are any issues remember this

#TODO update elevator map values


class Elevator(commands2.Subsystem):

    def __init__(self):
        commands2.CommandScheduler.registerSubsystem(self)
        self.main_motor = phoenix6.hardware.TalonFX(TunerConstants.elevator_main_id, "rio")
        self.follower_motor = phoenix6.hardware.TalonFX(TunerConstants.elevator_follower_id, "rio")
        self.follower_motor.set_control(phoenix6.controls.follower.Follower(TunerConstants.elevator_main_id, False))

        self.start_position = self.main_motor.get_position().value_as_double - 1
        self.up_limit = self.start_position + MoveElevatorToPosition.top # NOTE motor moves backwards to "up" limit is negative.
        self.prev_time = time.time()

        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self.nt_table = self.nt_instance.getTable("elevator")

        self.elevator_topic = self.nt_table.getDoubleTopic("elevator_position")
        self.elevator_publish = self.elevator_topic.publish()
        self.elevator_publish.set(0.0)

    def sim_update(self):
        self.prev_time = time.time()

    def move_command(self, speed: typing.Callable[[], float]) -> commands2.Command:
        return MoveElevatorCommand(self, speed)
    
    def change_height(self, speed: float):
        speed *= -1
        
        if self.get_position() <= self.up_limit and speed < 0:
            # Stop the motor if we went too high
            self.main_motor.set(0)
        elif self.get_position() >= self.start_position and speed > 0:
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

    def telemetry(self):
        box = wpilib.Mechanism2d(29.5, 29.5)
        stationary_root = box.getRoot("Elevator", 15, 0.75)

        elevator_position = -1 * self.get_position() * 30/1000.0
        #wrist_rotation = self.coral_wrist_sim.getPosition()

        platform_root = box.getRoot("Platform", 15, elevator_position)
        stationary_root.appendLigament(
            "stationary", 1, 90, 6, wpilib.Color8Bit(255, 255, 255)
        )
        root = platform_root.appendLigament(
            "platform", 1, 90
        )
        #root.appendLigament(
        #    "wrist", 1, 180 + wrist_rotation, 6, wpilib.Color8Bit(0, 0, 255)
        #)
        wpilib.SmartDashboard.putData("Elevator", box)

        self.elevator_publish.set(self.get_position())
        
    
    def get_position(self) -> float:
        return self.main_motor.get_position().value_as_double
    

class MoveElevatorCommand(commands2.Command):
    def __init__(self, elevator: Elevator, speed: typing.Callable[[], bool]):
        self.elevator = elevator
        self.speed = speed
        self.addRequirements(self.elevator)

        self.command_timer = wpilib.Timer()
        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.commands = self.ntcore_instance.getTable("commands")
        self.command_topic = self.commands.getFloatTopic("MoveElevator")
        self.command_publish = self.command_topic.publish()
        self.command_publish.set(0.0)

    def initialize(self):
        self.command_timer.reset()
        self.command_timer.start()
        
    def execute(self):
        self.elevator.change_height(self.speed())

        self.command_publish.set(self.command_timer.get())
        
    def end(self, interrupted):
        self.elevator.change_height(0)

class MoveElevatorToPosition(commands2.Command):
    # NOTE Moving the elevator up is actually negative values.
    top = -90
    middle = -55
    bottom = -27
    platform = -7.5
    def __init__(self, elevator: Elevator, position):
        self.elevator = elevator
        self.position = position
        self.addRequirements(self.elevator)

        self.command_timer = wpilib.Timer()
        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.commands = self.ntcore_instance.getTable("commands")
        self.command_topic = self.commands.getFloatTopic("MoveElevator")
        self.command_publish = self.command_topic.publish()
        self.command_publish.set(0.0)

    def initialize(self):
        self.command_timer.reset()
        self.command_timer.start()

    def execute(self):
        if self.elevator.get_position() < self.position:
            self.elevator.change_height(-0.5)
        else:
            self.elevator.change_height(0.5)

        self.command_publish.set(self.command_timer.get())

    def isFinished(self):
        return abs(self.elevator.get_position() - self.position) < 0.5
    
    def end(self, interrupted):
        self.elevator.change_height(0)
