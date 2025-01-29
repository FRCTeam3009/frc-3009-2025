from ntcore import NetworkTableInstance
import commands2.cmd

class Limelight(object):
    def __init__(self):
        self.nt_instance = NetworkTableInstance.create()
        self.table = self.nt_instance.getTable("limelight")
        self.botposetopic = self.table.getDoubleArrayTopic("botpose")
        self.botposesub = self.botposetopic.subscribe([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def update_command(self) -> commands2.Command:
        return commands2.cmd.run(lambda: self.update()).ignoringDisable(True)

    def update(self):
        self.currentvalue = self.botposesub.get()