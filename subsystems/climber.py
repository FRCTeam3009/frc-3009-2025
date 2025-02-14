import rev
import wpimath.system.plant
import commands2
from generated.tuner_constants import TunerConstants


class Climber:
    def __init__(self):
        self.climber_motor = rev.SparkMax(TunerConstants.climber_id, rev.SparkLowLevel.MotorType.kBrushless)
        self.climber_motor_sim = rev.SparkMaxSim(self.climber_motor, wpimath.system.plant.DCMotor.NEO(1))
        speed = 0.5

    def climber_movement(self, speed):
        self.climber_motor.set(speed)
        self.climber_motor_sim.setAppliedOutput(speed)

class MoveClimberCommand(commands2.Command):
    def __init__(self, climber: Climber, speed: float):
        self.climber = climber
        self.speed = speed

    def execute(self):
        self.climber.climber_movement(self.speed)

    def end(self, interrupted):
        self.climber.climber_movement(0)