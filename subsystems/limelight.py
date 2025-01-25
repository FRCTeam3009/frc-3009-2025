from ntcore import NetworkTableInstance
import commands2.cmd

class Limelight(object):
    def __init__(self):
        self.NetworkTable = NetworkTableInstance.getDefault()
        self.table = self.NetworkTable.getTable("limelight")
        self.botposetopic = self.table.getDoubleArrayTopic("botpose")
        self.botposesub = self.botposetopic.subscribe([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        commands2.cmd.run(lambda: self.update()).ignoringDisable(True).schedule()

    def update(self):
        self.currentvalue = self.botposesub.get()