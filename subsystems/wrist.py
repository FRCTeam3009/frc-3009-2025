import commands2
import phoenix5
import rev
import ntcore
import wpimath
import wpimath.system.plant
import wpilib
import typing
from generated.tuner_constants import TunerConstants

class Wrist(commands2.Subsystem):
    def __init__(self):
        commands2.CommandScheduler.registerSubsystem(self)
        self.coral_out_motor = phoenix5.TalonSRX(TunerConstants.coral_out_id)

        self.coral_wrist_motor = rev.SparkMax(TunerConstants.coral_wrist_id, rev.SparkLowLevel.MotorType.kBrushless)
        self.coral_wrist_sim = rev.SparkMaxSim(self.coral_wrist_motor, wpimath.system.plant.DCMotor.NEO(1))

        self.up_wrist_limit = 100
        self.down_wrist_limit = 0

        self.top_sensor = wpilib.DigitalInput(9)
        self.bottom_sensor = wpilib.DigitalInput(8)

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

    def coral_out(self, speed: float):
        self.coral_out_motor.set(phoenix5.TalonSRXControlMode.PercentOutput, speed)
        self.coral_out_motor.getSimCollection().addQuadraturePosition(round(speed * 10))
        self.coral_out_motor.getSimCollection().setQuadratureVelocity(round(speed * 10))

    def coral_wrist(self, speed: float):
        self.coral_wrist_motor.set(speed)
        self.coral_wrist_motor.set(speed)
        self.coral_wrist_sim.setAppliedOutput(speed)
        self.coral_wrist_sim.setPosition(self.coral_wrist_sim.getPosition() + speed * 2)
        self.coral_wrist_sim.getAbsoluteEncoderSim().setPosition(self.coral_wrist_sim.getPosition() + speed * 2)
        return
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
        self.wrist_publish.set([self.coral_wrist_motor.getEncoder().getPosition(), self.get_wrist_position()])
        self.coral_sensor_top_publish.set(self.top_sensor.get())
        self.coral_sensor_bottom_publish.set(self.bottom_sensor.get())

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

class CoralOutCommand(commands2.Command):
    def __init__(self, wrist: Wrist, speed: typing.Callable[[], float]):
        self.wrist = wrist
        self.speed = speed
        self.timer = wpilib.Timer()
        self.sensor = self.wrist.coral_sensor_shot
        self.addRequirements(self.wrist)

        self.command_timer = wpilib.Timer()
        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.commands = self.ntcore_instance.getTable("commands")
        self.command_topic = self.commands.getFloatTopic("CoralOut")
        self.command_publish = self.command_topic.publish()
        self.command_publish.set(0.0)

    def initialize(self):
        self.timer.reset()
        self.timer.start()
        self.command_timer.reset()
        self.command_timer.start()

    def execute(self):
        self.wrist.coral_out(self.speed())

        self.command_publish.set(self.command_timer.get())

    def isFinished(self):
        if self.sensor():
            return True
        elif self.timer.hasElapsed(1):
            return True
        return False

    def end(self, interrupted):
        self.wrist.coral_out(0)
        self.timer.stop()
        self.timer.reset()

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
    top = 30
    middle = 40
    bottom = 40
    platform = 50
    def __init__(self, wrist: Wrist, position):
        self.wrist = wrist
        self.position = position
        self.addRequirements(self.wrist)

        self.command_timer = wpilib.Timer()
        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.commands = self.ntcore_instance.getTable("commands")
        self.command_topic = self.commands.getFloatTopic("CoralWrist")
        self.command_publish = self.command_topic.publish()
        self.command_publish.set(0.0)

    def execute(self):
        if self.wrist.get_wrist_position() < self.position:
            self.wrist.coral_wrist(1)
        else:
            self.wrist.coral_wrist(-1)

        self.command_publish.set(self.command_timer.get())

    def isFinished(self):
        return abs(self.wrist.get_wrist_position() - self.position) < 5
    
    def end(self, interrupted):
        self.wrist.coral_wrist(0)

class coral_wait(commands2.Command):
    def __init__(self, sensor: typing.Callable[[], bool]):
        self.sensor = sensor
        self.timer = wpilib.Timer()
        self.leave_timer = wpilib.Timer()

    def initialize(self):
        self.leave_timer.reset()
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
