import commands2
import wpilib

class SomeCommand(commands2.Command):
    def __init__(self):
        self.timer = wpilib.Timer()

    def initialize(self):
        self.timer.start()
        
    def isFinished(self):
        return self.timer.hasElapsed(4)
    
    def execute(self):
        print("Stuff")