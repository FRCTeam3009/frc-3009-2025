import commands2
import phoenix5
import wpilib
import subsystems.wrist
import typing
import ntcore
from generated.tuner_constants import TunerConstants


class Shooter():
    def __init__(self):
        commands2.CommandScheduler.registerSubsystem(self)
        self.coral_out_motor = phoenix5.TalonSRX(TunerConstants.coral_out_id)

    def coral_out(self, speed: float):
        self.coral_out_motor.set(phoenix5.TalonSRXControlMode.PercentOutput, speed)
        self.coral_out_motor.getSimCollection().addQuadraturePosition(round(speed * 10))
        self.coral_out_motor.getSimCollection().setQuadratureVelocity(round(speed * 10))

class CoralOutCommand(commands2.Command):
    def __init__(self, shooter: Shooter, speed: typing.Callable[[], float]):
        self.shooter = shooter
        self.speed = speed
        self.timer = wpilib.Timer()
        self.sensor = subsystems.wrist.Wrist.coral_sensor_shot

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
        self.shooter.coral_out(self.speed())
        self.command_publish.set(self.command_timer.get())

    def end(self, interrupted):
        self.shooter.coral_out(0)
        self.timer.stop()
        self.timer.reset()