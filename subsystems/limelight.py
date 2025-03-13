from ntcore import NetworkTableInstance
import commands2.cmd
import phoenix6.swerve
import phoenix6.utils
import wpimath.geometry
import wpimath.units
import subsystems.command_swerve_drivetrain
import wpimath
import subsystems.drive_robot_relative
import subsystems.limelight
import subsystems.limelight_positions

class Limelight(object):
    def __init__(self, name: str, drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain):
        # [forward, horizontal, vertical, roll, pitch, yaw, latency, tag count, tag span, average distance, average area]

        self.nt_instance = NetworkTableInstance.getDefault()
        self.table = self.nt_instance.getTable(name)
        # NOTE Use pose2d_from_botpose() to make this easier to deal with.
        self.botposetopic = self.table.getDoubleArrayTopic("botpose_wpiblue")
        self.botposesub = self.botposetopic.subscribe([])
        self.drive_train = drive_train

        # NOTE Target pose in robot space returns [horizontal, vertical, forward, pitch, yaw, roll].
        # This is NOT CONSISTENT with the bot pose in field space values.
        # Use pose2d_from_targetpose() to make this easier to deal with.
        self.targetpose_botspacetopic = self.table.getDoubleArrayTopic("targetpose_robotspace")
        self.targetpose_botspacesub = self.targetpose_botspacetopic.subscribe([])

        self.smooth_botpose = subsystems.limelight_positions.SmoothPosition()
        self.smooth_targetpose = subsystems.limelight_positions.SmoothPosition()

        self.lined_up_topic = self.table.getBooleanTopic("test lined up")
        self.lined_up_publish = self.lined_up_topic.publish()
        self.lined_up_publish.set(False)

        self.bot_pose_topic = self.table.getDoubleArrayTopic("test current bot pose")
        self.bot_pose_publish = self.bot_pose_topic.publish()
        self.bot_pose_publish.set([0.0, 0.0, 0.0])

        self.offset_topic = self.table.getDoubleArrayTopic("lineup_offset")
        self.offset_pub = self.offset_topic.publish()
        self.offset_pub.set([0.0, 0.0, 0.0])

    def update_command(self) -> commands2.Command:
        return commands2.cmd.run(self.update).repeatedly().ignoringDisable(True)

    def update(self):
        botpose = self.botposesub.get()
        botpose2d = subsystems.limelight_positions.pose2d_from_targetpose(botpose)
        self.smooth_botpose.append_pose(botpose2d)

        targetpose = self.targetpose_botspacesub.get()
        targetpose2d = subsystems.limelight_positions.pose2d_from_targetpose(targetpose)
        self.smooth_targetpose.append_pose(targetpose2d)

    def odometry_command(self) -> commands2.Command:
        return commands2.cmd.run(self.odometry_update).repeatedly().ignoringDisable(True)
    
    def odometry_update(self):
        self.drive_train.add_vision_measurement(self.smooth_botpose.get_average_pose(), 0.05)
    
    def lined_up(self):
        if phoenix6.utils.is_simulation():
            return True

        pose = self.smooth_targetpose.get_average_pose()
        pose = subsystems.limelight_positions.correct_target_pose(pose)
        x_value = wpimath.units.metersToInches(pose.X())
        y_value = wpimath.units.metersToInches(pose.Y())
        rotation = pose.rotation().degrees()

        self.offset_pub.set([x_value, y_value, rotation])
        
        if abs(x_value - 26) <= 1 and abs(y_value) <= 1 and abs(rotation) <= 5:
            return True
        return False
    
    def telemetry(self):
        self.lined_up_publish.set(self.lined_up())
        pose = self.smooth_targetpose.get_average_pose()
        pose = subsystems.limelight_positions.correct_target_pose(pose)
        self.bot_pose_target_var = [wpimath.units.metersToInches(pose.X()), 
                                    wpimath.units.metersToInches(pose.Y()), 
                                    pose.rotation().degrees()]
        self.bot_pose_publish.set(self.bot_pose_target_var)

def lineup_apriltag_command(
        drivetrain : subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
        limelight : Limelight,
        ) -> commands2.Command:
    
    targetpose = limelight.smooth_targetpose.get_average_pose()
    targetpose = subsystems.limelight_positions.correct_target_pose(targetpose)
    x = targetpose.X() - wpimath.units.inchesToMeters(26)
    offset = wpimath.geometry.Transform2d(x, targetpose.Y(), targetpose.rotation())

    return subsystems.drive_robot_relative.DriveRobotRelativeCommand(drivetrain, offset, subsystems.drive_robot_relative.NORMAL_SPEED)
