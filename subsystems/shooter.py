import commands2
import rev
import wpilib
import subsystems.wrist
import typing
import ntcore
import wpimath.system.plant
from generated.tuner_constants import TunerConstants

SPEED = 1.0
AUTO_SPEED = 0.1

# NOTE the shooter wheel handles both Coral and Algae, just reversed for one of them.
class Shooter(commands2.Subsystem):
    def __init__(self):
        commands2.CommandScheduler.registerSubsystem(self)
        self.motor = rev.SparkMax(TunerConstants.coral_algae_id, rev.SparkLowLevel.MotorType.kBrushless)
        self.motor_sim = rev.SparkSim(self.motor, wpimath.system.plant.DCMotor.NEO(1))

    def move(self, speed: float):
        self.motor.set(speed)
        self.motor_sim.setAppliedOutput(speed)
        self.motor_sim.setPosition(self.motor_sim.getPosition() + speed * 2)
        self.motor_sim.getAbsoluteEncoderSim().setPosition(self.motor_sim.getPosition() + speed * 2)


class CoralOutCommand(commands2.Command):
    def __init__(self, shooter: Shooter, speed: typing.Callable[[], float]):
        self.addRequirements(shooter)
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
        self.shooter.move(self.speed())
        self.command_publish.set(self.command_timer.get())

    def end(self, interrupted):
        self.shooter.move(0)
        self.timer.stop()
        self.timer.reset()

class HalfShot(commands2.Command):
    def __init__(self, shooter: Shooter):
        self.addRequirements(shooter)
        self.shooter = shooter
        self.position = 0
    
    def initialize(self):
        self.position = self.shooter.motor.getEncoder().getPosition()

    def execute(self):
        self.shooter.motor.set(-0.5)

    def isFinished(self):
        current_pose = self.shooter.motor.getEncoder().getPosition()
        return current_pose <= self.position - 0.1
    
    def end(self, interrupted):
        self.shooter.motor.set(0)

class HoldShooter(commands2.Command):
    def __init__(self, shooter: Shooter):
        self.shooter = shooter
        self.addRequirements(self.shooter)
        self.position_to_hold = 0

    def initialize(self):
        self.position_to_hold = self.shooter.motor.getEncoder().getPosition()
    
    def execute(self):
        pos = self.shooter.motor.getEncoder().getPosition()
        if pos < self.position_to_hold:
            self.shooter.motor.set(0.01)
        elif pos > self.position_to_hold:
            self.shooter.motor.set(-0.01)
        else:
            self.shooter.motor.set(0)