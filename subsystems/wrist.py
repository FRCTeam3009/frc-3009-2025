import commands2
import rev
import ntcore
import wpimath
import wpimath.system.plant
import wpilib
import typing
import wpimath.controller
import wpimath.trajectory
import math
from wpimath.controller import PIDController, ArmFeedforward

import wpimath.units
from generated.tuner_constants import TunerConstants

HOLD_SPEED = 0.15
LOW_SPEED = 0.075
DRIVE_SPEED = 0.15
TURBO_SPEED = 1.0

class Wrist(commands2.Subsystem):
    def __init__(self):
        commands2.CommandScheduler.registerSubsystem(self)

        self.coral_wrist_motor = rev.SparkMax(TunerConstants.coral_wrist_id, rev.SparkLowLevel.MotorType.kBrushless)
        self.coral_wrist_sim = rev.SparkMaxSim(self.coral_wrist_motor, wpimath.system.plant.DCMotor.NEO(1))
        self.coral_wrist_sim.getAbsoluteEncoderSim().setPositionConversionFactor(360.0)

        self.up_wrist_limit = 125
        self.down_wrist_limit = CoralWristToPosition.pickup

        self.top_sensor = wpilib.DigitalInput(3)
        self.top_sensor2 = wpilib.DigitalInput(0)
        self.bottom_sensor = wpilib.DigitalInput(2)
        self.bottom_sensor2 = wpilib.DigitalInput(1)

        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self.nt_table = self.nt_instance.getTable("wrist")

        self.wrist_topic = self.nt_table.getDoubleArrayTopic("wrist_position")
        self.wrist_publish = self.wrist_topic.publish()
        self.wrist_publish.set([0.0, 0.0])

        self.coral_sensor_top_topic = self.nt_table.getBooleanTopic("coral_top")
        self.coral_sensor_top_publish = self.coral_sensor_top_topic.publish()
        self.coral_sensor_top_publish.set(False)

        self.coral_sensor_bottom_topic = self.nt_table.getBooleanTopic("coral_bottom")
        self.coral_sensor_bottom_publish = self.coral_sensor_bottom_topic.publish()
        self.coral_sensor_bottom_publish.set(False)

        self.coral_sensor_bottom2_topic = self.nt_table.getBooleanTopic("coral_bottom2")
        self.coral_sensor_bottom2_publish = self.coral_sensor_bottom2_topic.publish()
        self.coral_sensor_bottom2_publish.set(False)

        self.coral_sensor_top2_topic = self.nt_table.getBooleanTopic("coral_top2")
        self.coral_sensor_top2_publish = self.coral_sensor_top2_topic.publish()
        self.coral_sensor_top2_publish.set(False)

        self.intake_servo = wpilib.Servo(0)

        self.intake_servo_topic = self.nt_table.getDoubleTopic("intake servo")
        self.intake_servo_publish = self.intake_servo_topic.publish()
        self.intake_servo_publish.set(0.0)

        self.position_to_hold = 0.0

    def coral_wrist(self, speed: float):
        if self.get_wrist_position() > self.up_wrist_limit and speed > 0.0:
            self.coral_wrist_motor.set(0)
        elif self.get_wrist_position() < self.down_wrist_limit and speed < 0.0:
            self.coral_wrist_motor.set(0)
        else:
            self.coral_wrist_motor.set(speed)
            self.coral_wrist_sim.setAppliedOutput(speed)
            sim_encoder = self.coral_wrist_sim.getAbsoluteEncoderSim()
            sim_encoder.setPosition(sim_encoder.getPosition() + speed)

    def telemetry(self):
        self.wrist_publish.set([self.coral_wrist_motor.getEncoder().getPosition(), self.coral_wrist_motor.getAbsoluteEncoder().getPosition()])
        self.coral_sensor_top_publish.set(self.top_sensor.get())
        self.coral_sensor_bottom_publish.set(self.bottom_sensor.get())
        
        self.coral_sensor_top2_publish.set(self.top_sensor2.get())
        self.coral_sensor_bottom2_publish.set(self.bottom_sensor2.get())

        self.intake_servo_publish.set(self.intake_servo.getPosition())

    def get_wrist_position(self) -> float:
        # NOTE absolute position wraps around from 0 (i.e. -1 => 359 degrees)
        # (we set a factor of 2*pi on the SparkMax to give us radians)
        pos = self.coral_wrist_motor.getAbsoluteEncoder().getPosition()
        if pos > math.pi:
            pos -= math.pi
        return pos
    
    def coral_sensor_receive(self):
        if self.top_sensor.get() or self.bottom_sensor.get():
            return True
        return False
    
    def coral_sensor_shot(self):
        if not self.top_sensor.get() and not self.bottom_sensor.get():
            return True
        return False

class CoralWristToPosition(commands2.Command):
    # TODO convert to radians and measure values
    top = 45.0
    middle = 55.0
    bottom = 55.0
    platform = 72.0 # Platform is almost straight forward
    pickup = 0.0 # Pickup is straight down
    def __init__(self, wrist: Wrist, position: float):
        self.wrist = wrist
        self.position = position

    def execute(self):
        self.wrist.position_to_hold = self.position

    def isFinished(self):
        return True

class CoralWait(commands2.Command):
    def __init__(self, sensor: typing.Callable[[], bool]):
        self.sensor = sensor
        self.timer = wpilib.Timer()

    def execute(self):
        if self.sensor():
            self.timer.start()
        else:
            self.timer.reset()
            self.timer.stop()

    def isFinished(self):
        return self.timer.hasElapsed(0.5)

    def end(self, interrupted):
        self.timer.stop()
        self.timer.reset()

class HoldPositionCommand(commands2.Command):
    def __init__(self, wrist: Wrist):
        self.wrist = wrist
        self.addRequirements(self.wrist)

        self.wristPID = PIDController(0.75, 0, 0)
        self.wristFeedforward = ArmFeedforward(0, 0.15, 0)

    def execute(self):
        feedback = self.wristPID.calculate(self.wrist.get_wrist_position(), self.wrist.position_to_hold)
        # Feed forward expects straight outward to be 0 so offset by -90 degrees
        feedforward = self.wristFeedforward.calculate(self.wrist.get_wrist_position() - math.pi / 2, self.wrist.coral_wrist_motor.getAbsoluteEncoder().getVelocity())
        self.wrist.coral_wrist_motor.setVoltage(feedback + feedforward)
        #print("Voltage Applied: " + str(feedback + feedforward))

class MoveIntake(commands2.Command):
    pickup_pose = 1.0
    drop_pose = 0.0
    def __init__(self, servo: wpilib.Servo):
        self.intake_servo = servo

    def execute(self):
        pose = self.intake_servo.getPosition()
        if pose > 0.5:
            self.intake_servo.setPosition(MoveIntake.drop_pose)
        else:
            self.intake_servo.setPosition(MoveIntake.pickup_pose)

    def isFinished(self):
        return True
    
class IncrementWrist(commands2.Command):
    def __init__(self, wrist: Wrist, amt: typing.Callable[[], float]):
        self.amount = amt
        self.wrist = wrist

    def execute(self):
        self.wrist.position_to_hold += self.amount()