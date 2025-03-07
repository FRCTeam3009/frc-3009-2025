import commands2
import subsystems.shooter
import phoenix5

class AlgaeShooter(commands2.Subsystem):
    def __init__(self, coral_shooter: subsystems.shooter.Shooter):
        commands2.CommandScheduler.registerSubsystem(self)

        self.algae_shooter_motor = coral_shooter.coral_out_motor

    def shooter(self, speed: float):
        self.algae_shooter_motor.set(phoenix5.TalonSRXControlMode.PercentOutput, speed)

class ShootAlgae(commands2.Command):
    def __init__(self, algae_shooter: AlgaeShooter, speed: float):
        self.algae_shooter = algae_shooter
        self.speed = speed

    def execute(self):
        self.algae_shooter.shooter(self.speed)