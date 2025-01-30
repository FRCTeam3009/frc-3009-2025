from ntcore import NetworkTableInstance
import commands2.cmd
import phoenix6.utils
import wpimath.geometry
import subsystems.command_swerve_drivetrain
import wpimath
import time

class Limelight(object):
    def __init__(self, drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain):
        self.nt_instance = NetworkTableInstance.create()
        self.table = self.nt_instance.getTable("limelight")
        self.botposetopic = self.table.getDoubleArrayTopic("botpose")
        self.botposesub = self.botposetopic.subscribe([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # tx, ty, tz, rx, ry, rz
        self.drive_train = drive_train


    def update_command(self) -> commands2.Command:
        return commands2.cmd.run(self.update).ignoringDisable(True)

    def update(self):
        self.currentvalue = self.botposesub.get()

    def odometry_command(self) -> commands2.Command:
        return commands2.cmd.run(self.odometry_update).ignoringDisable(True)
    
    def odometry_update(self):
        botpose = self.botposesub.get()
        if self.list_check(botpose):
            pose2d = wpimath.geometry.Pose2d(botpose[0], botpose[1], botpose[5])
            self.drive_train.add_vision_measurement(pose2d, time.time())

    def list_check(self, list):
        if len(list) < 6:
            return False

        for value in list:
            if value != 0.0:
                return True
            
        return False
            