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

# TODO drill holes in things to make weight
# TODO cut some aluminum to make weight
# TODO test auto modes for tracking specific april tags

SPEED = 2.0

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

        self.p_topic = self.nt_table.getDoubleTopic("P")
        self.p_subscribe = self.p_topic.subscribe(0.15)
        self.p_publish = self.p_topic.publish()
        self.p_publish.set(0.15)
        self.i_topic = self.nt_table.getDoubleTopic("I")
        self.i_subscribe = self.i_topic.subscribe(0.08)
        self.i_publish = self.i_topic.publish()
        self.i_publish.set(0.08)
        self.d_topic = self.nt_table.getDoubleTopic("D")
        self.d_subscribe = self.d_topic.subscribe(0.0)
        self.d_publish = self.d_topic.publish()
        self.d_publish.set(0)

        
        self.s_topic = self.nt_table.getDoubleTopic("Ks")
        self.s_subscribe = self.s_topic.subscribe(0.1)
        self.s_publish = self.s_topic.publish()
        self.s_publish.set(0.1)
        self.g_topic = self.nt_table.getDoubleTopic("Kg")
        self.g_subscribe = self.g_topic.subscribe(0.05)
        self.g_publish = self.g_topic.publish()
        self.g_publish.set(0.05)
        self.v_topic = self.nt_table.getDoubleTopic("Kv")
        self.v_subscribe = self.v_topic.subscribe(0.07)
        self.v_publish = self.v_topic.publish()
        self.v_publish.set(0.07)
        self.a_topic = self.nt_table.getDoubleTopic("Ka")
        self.a_subscribe = self.a_topic.subscribe(0.02)
        self.a_publish = self.a_topic.publish()
        self.a_publish.set(0.02)

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
        if (self.elevator.position_to_hold < MoveElevatorToPosition.lower_limit):
            self.elevator.position_to_hold = MoveElevatorToPosition.lower_limit
        elif (self.elevator.position_to_hold > MoveElevatorToPosition.upper_limit):
            self.elevator.position_to_hold = MoveElevatorToPosition.upper_limit


class MoveElevatorToPosition(commands2.Command):
    upper_limit = 84
    lower_limit = -12
    L4 = upper_limit
    L3 = 40
    L2 = 9.5
    L1 = lower_limit
    pickup = 1.0
    auto_pose = 10
    
    def __init__(self, elevator: Elevator, position : float):
        self.elevator = elevator
        self.position = position

    def execute(self):
        self.elevator.position_to_hold = self.position
        if (self.elevator.position_to_hold < MoveElevatorToPosition.lower_limit):
            self.elevator.position_to_hold = MoveElevatorToPosition.lower_limit
        elif (self.elevator.position_to_hold > MoveElevatorToPosition.upper_limit):
            self.elevator.position_to_hold = MoveElevatorToPosition.upper_limit

    def isFinished(self):
        return abs(self.elevator.get_position() - self.elevator.position_to_hold) < 0.5

class HoldPositionCommand(commands2.Command):
    def __init__(self, elevator: Elevator):
        self.elevator = elevator
        self.addRequirements(self.elevator)

        self.elevatorPID = wpimath.controller.PIDController(0.1, 0.08, 0)
        self.elevatorFeedForward = wpimath.controller.ElevatorFeedforward(0.1, 0.05, 0.07, 0.02)
    
    def execute(self):
        feedback = self.elevatorPID.calculate(self.elevator.get_position(), self.elevator.position_to_hold)
        feedforward = self.elevatorFeedForward.calculate(self.elevator.main_motor.get_velocity().value_as_double)
        self.elevator.main_motor.setVoltage(feedback + feedforward)