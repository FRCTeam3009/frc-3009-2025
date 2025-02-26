import rev
import wpimath.system.plant
import commands2
import ntcore
import wpilib
from generated.tuner_constants import TunerConstants


class Climber(commands2.Subsystem):
    def __init__(self):
        commands2.CommandScheduler.registerSubsystem(self)
        self.climber_motor = rev.SparkFlex(TunerConstants.climber_id, rev.SparkLowLevel.MotorType.kBrushless)
        self.climber_motor_sim = rev.SparkFlexSim(self.climber_motor, wpimath.system.plant.DCMotor.NEO(1))

        self.down_limit = self.get_position() + 5
        self.up_limit = self.down_limit + 550

        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self.nt_table = self.nt_instance.getTable("climber")
        self.motor_topic = self.nt_table.getDoubleArrayTopic("motor")
        self.motor_publish = self.motor_topic.publish()
        self.motor_publish.set([0.0, 0.0])

    def climber_movement(self, speed: float):
        if self.get_position() > self.up_limit and speed > 0:
            self.climber_motor.set(0)
        elif self.get_position() < self.down_limit and speed < 0:
            self.climber_motor.set(0)
        else:
            self.climber_motor.set(speed)
            self.climber_motor_sim.setAppliedOutput(speed)

    def get_position(self):
        return self.climber_motor.getEncoder().getPosition()
    
    def telemetry(self):
        positions = [
            self.climber_motor.getEncoder().getPosition(), 
            self.climber_motor.getAbsoluteEncoder().getPosition(),
            ]
        self.motor_publish.set(positions)

class MoveClimberCommand(commands2.Command):
    def __init__(self, climber: Climber, speed: float):
        self.climber = climber
        self.speed = speed
        self.addRequirements(self.climber)

        self.command_timer = wpilib.Timer()
        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.commands = self.ntcore_instance.getTable("commands")
        self.command_topic = self.commands.getFloatTopic("MoveClimber")
        self.command_publish = self.command_topic.publish()
        self.command_publish.set(0.0)

    def initialize(self):
        self.command_timer.reset()
        self.command_timer.start()

    def execute(self):
        self.climber.climber_movement(self.speed)
        
        self.command_publish.set(self.command_timer.get())

    def end(self, interrupted):
        self.climber.climber_movement(0)
