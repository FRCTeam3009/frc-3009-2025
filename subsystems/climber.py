import rev
import wpimath.system.plant
import commands2
from generated.tuner_constants import TunerConstants


class Climber:
    def __init__(self):
        self.climber_motor = rev.SparkMax(TunerConstants.climber_id, rev.SparkLowLevel.MotorType.kBrushless)
        self.climber_motor_sim = rev.SparkMaxSim(self.climber_motor, wpimath.system.plant.DCMotor.NEO(1))
        speed = 0.5

        self.down_limit = self.get_position()
        self.up_limit = self.down_limit + 100

    def climber_movement(self, speed):
        if self.get_position() > self.up_limit and speed > 0:
            self.climber_motor.set(0)
        elif self.get_position() < self.down_limit and speed < 0:
            self.climber_motor.set(0)
        else:
            self.climber_motor.set(speed)
            self.climber_motor_sim.setAppliedOutput(speed)

    def get_position(self):
        return self.climber_motor.getEncoder().getPosition()

class MoveClimberCommand(commands2.Command):
    def __init__(self, climber: Climber, speed: float):
        self.climber = climber
        self.speed = speed

    def execute(self):
        self.climber.climber_movement(self.speed)

    def end(self, interrupted):
        self.climber.climber_movement(0)