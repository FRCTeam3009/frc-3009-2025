from ntcore import NetworkTableInstance
import commands2.cmd
import wpimath.geometry
import wpimath.units
import subsystems.command_swerve_drivetrain
import wpimath
import subsystems.drive_robot_relative
import subsystems.limelight
import subsystems.limelight_positions
import json

APRIL_TAG_OFFSET = 0.56
CORAL_OFFSET = wpimath.units.inchesToMeters(-2.5)

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

        self.json_topic = self.table.getStringTopic("json")
        self.json_subscribe = self.json_topic.subscribe("")
        self.target_poses = {6: subsystems.limelight_positions.SmoothPosition(),
                            7: subsystems.limelight_positions.SmoothPosition(),
                            8: subsystems.limelight_positions.SmoothPosition(),
                            9: subsystems.limelight_positions.SmoothPosition(),
                            10: subsystems.limelight_positions.SmoothPosition(),
                            11: subsystems.limelight_positions.SmoothPosition(),
                            17: subsystems.limelight_positions.SmoothPosition(),
                            18: subsystems.limelight_positions.SmoothPosition(),
                            19: subsystems.limelight_positions.SmoothPosition(),
                            20: subsystems.limelight_positions.SmoothPosition(),
                            21: subsystems.limelight_positions.SmoothPosition(),
                            22: subsystems.limelight_positions.SmoothPosition(),
                            1: subsystems.limelight_positions.SmoothPosition(),
                            2: subsystems.limelight_positions.SmoothPosition(),
                            12: subsystems.limelight_positions.SmoothPosition(),
                            13: subsystems.limelight_positions.SmoothPosition()}

        self.smooth_botpose = subsystems.limelight_positions.SmoothPosition()

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

        jsonStr = self.json_subscribe.get()
        if jsonStr is None or jsonStr == "":
            for key in self.target_poses.keys():
                val = wpimath.geometry.Pose2d(0.0, 0.0, 0.0)
                self.target_poses[key].append_pose(val)
            return
        
        self.parsed = json.loads(jsonStr)
        for key in self.target_poses.keys():
            saw_tag = False
            for f in self.parsed["Fiducial"]:
                id = f["fID"]
                if id == key:
                    val = subsystems.limelight_positions.pose2d_from_targetpose(f["t6t_rs"])
                    self.target_poses[key].append_pose(val)
                    saw_tag = True
            if not saw_tag:
                val = wpimath.geometry.Pose2d(0.0, 0.0, 0.0)
                self.target_poses[key].append_pose(val)


    def odometry_command(self) -> commands2.Command:
        return commands2.cmd.run(self.odometry_update).repeatedly().ignoringDisable(True)
    
    def odometry_update(self):
        self.drive_train.add_vision_measurement(self.smooth_botpose.get_average_pose(), 0.05)
    
    def telemetry(self):
        pose = self.target_poses[19].get_average_pose()
        pose = subsystems.limelight_positions.correct_target_pose(pose)
        self.bot_pose_target_var = [wpimath.units.metersToInches(pose.X()), 
                                    wpimath.units.metersToInches(pose.Y()), 
                                    pose.rotation().degrees()]
        self.bot_pose_publish.set(self.bot_pose_target_var)

class lineupCommand(commands2.Command):
    def __init__(self, drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, limelight: Limelight, april_id: int):
        self.drivetrain = drivetrain
        self.limelight = limelight
        self.april_id = april_id
        self.command = commands2.Command()

    def initialize(self):
        tgtPose = self.limelight.target_poses[self.april_id]
        if (tgtPose is None or tgtPose.is_zero()):
            self.command = subsystems.drive_robot_relative.DriveRobotRelativeCommand(self.drivetrain, wpimath.geometry.Transform2d(0, 0, 0), 0)
            self.command.initialize()
            return
        targetpose = tgtPose.get_average_pose()
        targetpose = subsystems.limelight_positions.correct_target_pose(targetpose)
        x = targetpose.X() - APRIL_TAG_OFFSET
        y = targetpose.Y() - CORAL_OFFSET
        offset = wpimath.geometry.Transform2d(x, y, targetpose.rotation())
        self.command = subsystems.drive_robot_relative.DriveRobotRelativeCommand(self.drivetrain, offset, subsystems.drive_robot_relative.NORMAL_SPEED)
        self.command.initialize()
    
    def execute(self):
        self.command.execute()
    
    def isFinished(self):
        return self.command.isFinished()