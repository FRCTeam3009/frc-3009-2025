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
from generated.tuner_constants import TunerConstants

# TODO we'll need to move the elevator to specific positions while driving up to the coral station.

# TODO we'll want to test the april tag odometry to see if we need to customize it at all
# (e.g. does it think we teleport around the field or does it update pretty smoothly.)

# TODO test the auto modes on a full size field.

# TODO errors with the robot relative drive if there are any issues remember this

ELEVATOR_TOP = 50
WRIST_TOP = 50

class Elevator(object):

    def __init__(self):
        self.main_motor = phoenix6.hardware.TalonFX(TunerConstants.elevator_main_id, "rio")
        self.follower_motor = phoenix6.hardware.TalonFX(TunerConstants.elevator_follower_id, "rio")

        self.coral_out_motor = phoenix5.TalonSRX(TunerConstants.coral_out_id)

        self.coral_wrist_motor = rev.SparkMax(TunerConstants.coral_wrist_id, rev.SparkLowLevel.MotorType.kBrushless)
        self.coral_wrist_sim = rev.SparkMaxSim(self.coral_wrist_motor, wpimath.system.plant.DCMotor.NEO(1))

        self.follower_motor.set_control(phoenix6.controls.follower.Follower(TunerConstants.elevator_main_id, False))

        self.start_position = self.main_motor.get_position().value_as_double
        self.up_limit = self.start_position + 1000.0
        self.prev_time = time.time()

        self.up_wrist_limit = 100
        self.down_wrist_limit = 0

        self.top_sensor = wpilib.DigitalInput(9)
        self.bottom_sensor = wpilib.DigitalInput(8)

    def sim_update(self):
        self.prev_time = time.time()

    def move_command(self, speed) -> commands2.Command:
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

    def coral_out(self, speed):
        self.coral_out_motor.set(phoenix5.TalonSRXControlMode.PercentOutput, speed)
        self.coral_out_motor.getSimCollection().addQuadraturePosition(round(speed * 10))
        self.coral_out_motor.getSimCollection().setQuadratureVelocity(round(speed * 10))

    def coral_wrist(self, speed):
        if self.get_wrist_position() > self.up_wrist_limit and speed > 0:
            self.coral_wrist_motor.set(0)
        elif self.get_wrist_position() < self.down_wrist_limit and speed < 0:
            self.coral_wrist_motor.set(0)
        else:
            self.coral_wrist_motor.set(speed)
            self.coral_wrist_sim.setAppliedOutput(speed)
            self.coral_wrist_sim.setPosition(self.coral_wrist_sim.getPosition() + speed * 2)
            self.coral_wrist_sim.getAbsoluteEncoderSim().setPosition(self.coral_wrist_sim.getPosition() + speed * 2)

    def telemetry(self):
        box = wpilib.Mechanism2d(29.5, 29.5)
        stationary_root = box.getRoot("Elevator", 15, 0.75)

        elevator_position = self.get_position() * 30/1000.0
        wrist_rotation = self.coral_wrist_sim.getPosition()

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
        
    
    def get_position(self) -> float:
        return self.main_motor.get_position().value_as_double

    def get_wrist_position(self) -> float:
        return self.coral_wrist_motor.getAbsoluteEncoder().getPosition()
    
    def coral_sensor_receive(self):
        if self.top_sensor.get() or self.bottom_sensor.get():
            return True
        return False
    
    def coral_sensor_shot(self):
        if not self.top_sensor.get() and not self.bottom_sensor.get():
            return True
        return False
    

class MoveElevatorCommand(commands2.Command):
    def __init__(self, elevator: Elevator, speed: typing.Callable[[], bool]):
        self.elevator = elevator
        self.speed = speed
        
    def execute(self):
        self.elevator.change_height(self.speed())
        
    def end(self, interrupted):
        self.elevator.change_height(0)

class CoralOutCommand(commands2.Command):
    def __init__(self, elevator: Elevator, speed: float):
        self.elevator = elevator
        self.speed = speed
        self.timer = wpilib.Timer()
        self.sensor = self.elevator.coral_sensor_shot

    def initialize(self):
        self.timer.start()

    def execute(self):
        self.elevator.coral_out(self.speed)

    def isFinished(self):
        if self.sensor():
            return True
        elif self.timer.hasElapsed(1):
            return True
        return False

    def end(self, interrupted):
        self.elevator.coral_out(0)
        self.timer.stop()
        self.timer.reset()

class coralWristCommand(commands2.Command):
    def __init__(self, elevator: Elevator, speed: typing.Callable[[], bool]):
        self.elevator = elevator
        self.speed = speed
    
    def execute(self):
        self.elevator.coral_wrist(self.speed())
    
    def end(self, interrupted):
        self.elevator.coral_wrist(0)

class coralWristToPosition(commands2.Command):
    def __init__(self, elevator: Elevator, position):
        self.elevator = elevator
        self.position = position

    def execute(self):
        if self.elevator.get_wrist_position() < self.position:
            self.elevator.coral_wrist(1)
        else:
            self.elevator.coral_wrist(-1)

    def isFinished(self):
        return abs(self.elevator.get_wrist_position() - self.position) < 10
    
    def end(self, interrupted):
        self.elevator.coral_wrist(0)

class MoveElevatorToPosition(commands2.Command):
    def __init__(self, elevator: Elevator, position):
        self.elevator = elevator
        self.position = position

    def execute(self):
        if self.elevator.get_position() < self.position:
            self.elevator.change_height(0.5)
        else:
            self.elevator.change_height(-0.5)

    def isFinished(self):
        return abs(self.elevator.get_position() - self.position) < 10
    
    def end(self, interrupted):
        self.elevator.change_height(0)

class coral_wait(commands2.Command):
    def __init__(self, sensor: typing.Callable[[], bool]):
        self.sensor = sensor
        self.timer = wpilib.Timer()
        self.leave_timer = wpilib.Timer()

    def initialize(self):
        self.leave_timer.start()

    def execute(self):
        print(self.sensor())

    def isFinished(self):
        if self.leave_timer.hasElapsed(5):
            return True
        
        if not self.sensor():
            self.timer.stop()
            self.timer.reset()
        elif self.sensor():
            self.timer.start()
            if self.timer.hasElapsed(0.5):
                self.timer.stop()
                self.timer.reset()
                return True
        return False

    def end(self, interrupted):
        pass
