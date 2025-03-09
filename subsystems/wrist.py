import commands2
import phoenix5
import rev
import ntcore
import wpimath
import wpimath.system.plant
import wpilib
import typing
from generated.tuner_constants import TunerConstants

SPEED = 0.15

class Wrist(commands2.Subsystem):
    def __init__(self):
        commands2.CommandScheduler.registerSubsystem(self)

        self.coral_wrist_motor = rev.SparkMax(TunerConstants.coral_wrist_id, rev.SparkLowLevel.MotorType.kBrushless)
        self.coral_wrist_sim = rev.SparkMaxSim(self.coral_wrist_motor, wpimath.system.plant.DCMotor.NEO(1))

        self.up_wrist_limit = 0.11
        self.down_wrist_limit = 0.0

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

    def coral_wrist(self, speed: float):
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
        self.wrist_publish.set([self.coral_wrist_motor.getEncoder().getPosition(), self.coral_wrist_motor.getAbsoluteEncoder().getPosition()])
        self.coral_sensor_top_publish.set(self.top_sensor.get())
        self.coral_sensor_bottom_publish.set(self.bottom_sensor.get())
        
        self.coral_sensor_top2_publish.set(self.top_sensor2.get())
        self.coral_sensor_bottom2_publish.set(self.bottom_sensor2.get())

    def get_wrist_position(self) -> float:
        return self.coral_wrist_motor.getEncoder().getPosition()
    
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
    #TODO update these values
    top = 0.02
    middle = 0.065
    bottom = 0.065
    platform = 0.10
    pickup = 0.10
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
        motor_power = difference * 3
        self.wrist.coral_wrist(motor_power)
