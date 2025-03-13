import commands2
import phoenix5
import rev
import ntcore
import wpimath
import wpimath.system.plant
import wpilib
import typing
import numpy
from generated.tuner_constants import TunerConstants

SPEED = 0.15

RANGE = 90.0 # About 90 degrees of motion

class Wrist(commands2.Subsystem):
    def __init__(self):
        commands2.CommandScheduler.registerSubsystem(self)

        self.coral_wrist_motor = rev.SparkMax(TunerConstants.coral_wrist_id, rev.SparkLowLevel.MotorType.kBrushless)
        self.coral_wrist_sim = rev.SparkMaxSim(self.coral_wrist_motor, wpimath.system.plant.DCMotor.NEO(1))

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


    def coral_wrist(self, speed: float):
        if self.get_wrist_position() > self.up_wrist_limit and speed > 0.0:
            self.coral_wrist_motor.set(0)
        elif self.get_wrist_position() < self.down_wrist_limit and speed < 0.0:
            self.coral_wrist_motor.set(0)
        else:
            self.coral_wrist_motor.set(speed)
            self.coral_wrist_sim.setAppliedOutput(speed)
            self.coral_wrist_sim.setPosition(self.coral_wrist_sim.getPosition() + speed * 2)
            self.coral_wrist_sim.getAbsoluteEncoderSim().setPosition(self.coral_wrist_sim.getPosition() + speed * 2)

    def telemetry(self):
        self.wrist_publish.set([self.coral_wrist_motor.getEncoder().getPosition(), self.coral_wrist_motor.getAbsoluteEncoder().getPosition()])
        self.coral_sensor_top_publish.set(self.top_sensor.get())
        self.coral_sensor_bottom_publish.set(self.bottom_sensor.get())
        
        self.coral_sensor_top2_publish.set(self.top_sensor2.get())
        self.coral_sensor_bottom2_publish.set(self.bottom_sensor2.get())

        self.intake_servo_publish.set(self.intake_servo.getPosition())

    def get_wrist_position(self) -> float:
        # NOTE absolute position wraps around from 0 to 360 (i.e. 361 => 1 and -1 => 359)
        # (we set a factor of 360 on the SparkMax to give us degrees of motion)
        pos = self.coral_wrist_motor.getAbsoluteEncoder().getPosition()
        if pos > 300:
            pos -= 360
        return pos
    
    def coral_sensor_receive(self):
        if self.top_sensor.get() or self.bottom_sensor.get():
            return True
        return False
    
    def coral_sensor_shot(self):
        if not self.top_sensor.get() and not self.bottom_sensor.get():
            return True
        return False

class CoralWristCommand(commands2.Command):
    def __init__(self, wrist: Wrist, speed: typing.Callable[[], bool]):
        self.wrist = wrist
        self.speed = speed
        self.addRequirements(self.wrist)

        self.command_timer = wpilib.Timer()
        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.commands = self.ntcore_instance.getTable("commands")
        self.command_topic = self.commands.getFloatTopic("CoralWrist")
        self.command_publish = self.command_topic.publish()
        self.command_publish.set(0.0)

    def initialize(self):
        self.command_timer.start()
    
    def execute(self):
        self.wrist.coral_wrist(self.speed())

        self.command_publish.set(self.command_timer.get())
    
    def end(self, interrupted):
        self.wrist.coral_wrist(0)

class CoralWristToPosition(commands2.Command):
    top = 45.0
    middle = 60.0
    bottom = 62.5
    platform = 72.0 # Platform is almost straight forward
    pickup = 0.0 # Pickup is straight down
    def __init__(self, wrist: Wrist, position: float, speed: float):
        self.wrist = wrist
        self.position = position
        self.addRequirements(self.wrist)
        self.speed = speed

        self.command_timer = wpilib.Timer()
        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.commands = self.ntcore_instance.getTable("commands")
        self.command_topic = self.commands.getFloatTopic("CoralWrist")
        self.command_publish = self.command_topic.publish()
        self.command_publish.set(0.0)

    def execute(self):
        if self.wrist.get_wrist_position() < self.position:
            self.wrist.coral_wrist(self.speed)
        else:
            self.wrist.coral_wrist(self.speed)

        self.command_publish.set(self.command_timer.get())

    def isFinished(self):
        return abs(self.wrist.get_wrist_position() - self.position) < 0.005
    
    def end(self, interrupted):
        self.wrist.coral_wrist(0)

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

    def initialize(self):
        self.position_to_hold = self.wrist.get_wrist_position()
    
    def execute(self):
        difference = self.position_to_hold - self.wrist.get_wrist_position()
        motor_power = difference / RANGE
        self.wrist.coral_wrist(motor_power)

class MoveIntake(commands2.Command):
    pickup_pose = 1.0
    drop_pose = 0.0
    def __init__(self, servo: wpilib.Servo):
        self.intake_servo = servo
        #self.intake_servo.setPosition(MoveIntake.drop_pose)

    def execute(self):
        pose = self.intake_servo.getPosition()
        if pose > 0.5:
            self.intake_servo.setPosition(MoveIntake.drop_pose)
        else:
            self.intake_servo.setPosition(MoveIntake.pickup_pose)

    def isFinished(self):
        return True