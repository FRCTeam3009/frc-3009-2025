import wpilib
import commands2

class Solenoids:
    def __init__(self):
        self.doubleSolenoid = wpilib.DoubleSolenoid(
            moduleType=wpilib.PneumaticsModuleType.CTREPCM,
            forwardChannel=0,
            reverseChannel=1,
        )
        self.doubleSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)

    def fire(self):
        self.doubleSolenoid.toggle()


    #timed fire and timed retract are part of timed move command
    def timed_fire(self):
        self.doubleSolenoid.set(wpilib.DoubleSolenoid.Value.kForward)

    def timed_retract(self):
        self.doubleSolenoid.set(wpilib.DoubleSolenoid.Value.kReverse)


    
class SolenoidsMoveCommand(commands2.Command):
    def __init__(self, solenoids: Solenoids):
        self.solenoids = solenoids

    def isFinished(self):
        return True

    def execute(self):
        self.solenoids.fire()

    def end(self, interrupted):
        pass


#isn't used yet because of possible danger with testing
class SolenoidsMoveCommandTimed(commands2.Command):
    def __init__(self, solenoids: Solenoids):
        self.solenoids = solenoids
        self.timer = wpilib.Timer()
        self.timer.start()

    def execute(self):
        self.solenoids.timed_fire()

    def isFinished(self):
        if self.timer.hasElapsed(2):
            return True
        else:
            return False

    def end(self, interrupted):
        self.solenoids.timed_retract()