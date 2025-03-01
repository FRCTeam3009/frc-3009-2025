from ntcore import NetworkTableInstance
import commands2.cmd
import phoenix6.swerve
import phoenix6.utils
import wpimath.geometry
import wpimath.units
import subsystems.command_swerve_drivetrain
import wpimath
import time
import wpilib

CORAL_POST_OFFSET = wpimath.units.inchesToMeters(6.47) # inches offset from center of AprilTag


class Limelight(object):
    def __init__(self, name: str, drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain):
        default_value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # tx, ty, tz, rx, ry, rz, latency, tag count, tag span, average distance, average area


        self.nt_instance = NetworkTableInstance.getDefault()
        self.table = self.nt_instance.getTable(name)
        self.botposetopic = self.table.getDoubleArrayTopic("botpose")
        self.botposesub = self.botposetopic.subscribe(default_value)
        self.drive_train = drive_train

        self.current_bot_pose_field = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_target_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.targetpose_botspacetopic = self.table.getDoubleArrayTopic("targetpose_robotspace")
        self.targetpose_botspacesub = self.targetpose_botspacetopic.subscribe([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # tx, ty, tz, rx, ry, rz

        self.drive_robot_relative = (
            phoenix6.swerve.requests.RobotCentric()
            .with_drive_request_type(
                phoenix6.swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )

        self.lined_up_topic = self.table.getBooleanTopic("test lined up")
        self.lined_up_publish = self.lined_up_topic.publish()
        self.lined_up_publish.set(False)

        self.bot_pose_topic = self.table.getDoubleArrayTopic("test current bot pose")
        self.bot_pose_publish = self.bot_pose_topic.publish()
        self.bot_pose_publish.set([0.0, 0.0, 0.0])

    def update_command(self) -> commands2.Command:
        return commands2.cmd.run(self.update).repeatedly().ignoringDisable(True)

    def update(self):
        fieldspace = self.botposesub.get()
        if self.list_check(fieldspace):
            self.current_bot_pose_field = fieldspace

        targetpose = self.targetpose_botspacesub.get()
        if self.list_check(targetpose):
            self.current_target_pose = targetpose

    def odometry_command(self) -> commands2.Command:
        return commands2.cmd.run(self.odometry_update).repeatedly().ignoringDisable(True)
    
    def odometry_update(self):
        botpose = self.current_bot_pose_field
        if self.list_check(botpose):
            pose2d = wpimath.geometry.Pose2d(botpose[0], botpose[1], botpose[5])
            seconds = 0
            if len(botpose) > 6:
                latency = botpose[6]
                latency = latency / 1000.0
                seconds = time.time() - latency
            self.drive_train.add_vision_measurement(pose2d, seconds)

    def list_check(self, l):
        if len(l) < 6:
            return False

        for value in l:
            if value != 0.0:
                return True
            
        return False

    
    def lined_up(self):
        x_value = wpimath.units.metersToInches(self.current_target_pose[2])
        y_value = wpimath.units.metersToInches(self.current_target_pose[0])
        rotation = self.current_target_pose[4]
        if phoenix6.utils.is_simulation():
            return True
        
        if self.list_check(self.current_target_pose) and abs(x_value - 26) <= 1 and abs(y_value) <= 1 and abs(rotation) <= 5:
            return True
        return False
    
    def telemetry(self):
        self.lined_up_publish.set(self.lined_up())
        self.bot_pose_target_var = [wpimath.units.metersToInches(self.current_target_pose[2]), 
                                    wpimath.units.metersToInches(self.current_target_pose[0]), 
                                    self.current_target_pose[4]]
        self.bot_pose_publish.set(self.bot_pose_target_var)
        
            
        

class LineUpAprilTagCommand(commands2.Command):

    def __init__(self,
                 drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                 limelight: Limelight,
                ):
        self.drive_train = drive_train
        self.limelight = limelight

    def execute(self):
        x = self.limelight.current_target_pose[2]
        y = self.limelight.current_target_pose[0]
        r = self.limelight.current_target_pose[4]
        forward = 0
        horizontal = 0
        rotation = 0
        if x > 0:
            forward = 0.2
        elif x < 0:
            forward = -0.2
        if y > 0:
            horizontal = 0.2
        elif y < 0:
            horizontal = -0.2
        if r < 5:
            rotation = 0.5
        elif r > 5:
            rotation = -0.5

        drive_request = lambda: self.limelight.drive_robot_relative.with_velocity_x(forward).with_velocity_y(horizontal).with_rotational_rate(rotation)
        self.drive_train.apply_request(drive_request).schedule()

    
    def isFinished(self):
        return self.limelight.lined_up()
    
    def end(self, interrupted):
        drive_request = lambda: self.limelight.drive_robot_relative.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(0)
        self.drive_train.apply_request(drive_request).schedule()

def line_up_command(drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain,
                    limelight: Limelight,
                            ):
    return LineUpAprilTagCommand(drive_train, limelight)

class DriveStraightCommand(commands2.Command):

    def __init__(self, 
                 drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                 limelight: Limelight,
                 distance: float,
                 speed: float
                 ):
        self.drive_train = drive_train
        self.limelight = limelight
        self.distance = distance
        self.speed = speed

    def initialize(self):
        self.start_position = self.drive_train.get_state().pose.X()
        self.end_position = self.start_position + self.distance

    def execute(self):
        drive_request = lambda: self.limelight.drive_robot_relative.with_velocity_x(self.speed).with_velocity_y(0).with_rotational_rate(0)
        self.drive_train.apply_request(drive_request).schedule()

    
    def isFinished(self):
        diff = self.end_position - self.drive_train.get_state().pose.X()
        return abs(diff) < wpimath.units.inchesToMeters(1)
    
    def end(self, interrupted):
        drive_request = lambda: self.limelight.drive_robot_relative.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(0)
        self.drive_train.apply_request(drive_request).schedule()

class DriveSidewaysCommand(commands2.Command):

    def __init__(self, 
                 drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                 limelight: Limelight,
                 distance: float,
                 speed: float
                 ):
        self.drive_train = drive_train
        self.limelight = limelight
        self.distance = distance
        self.speed = speed

    def initialize(self):
        self.start_position = self.drive_train.get_state().pose.Y()
        self.end_position = self.start_position + self.distance

    def execute(self):
        drive_request = lambda: self.limelight.drive_robot_relative.with_velocity_x(0).with_velocity_y(self.speed).with_rotational_rate(0)
        self.drive_train.apply_request(drive_request).schedule()

    
    def isFinished(self):
        diff = self.end_position - self.drive_train.get_state().pose.Y()
        print(str(self.start_position) + " " + str(self.end_position) + " " + str(self.drive_train.get_state().pose.Y()))
        return abs(diff) < wpimath.units.inchesToMeters(1)
    
    def end(self, interrupted):
        drive_request = lambda: self.limelight.drive_robot_relative.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(0)
        self.drive_train.apply_request(drive_request).schedule()


def drive_forward_command(drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain,
                          limelight: Limelight):
    distance = wpimath.units.inchesToMeters(11.0)
    speed = 0.5
    return DriveStraightCommand(drive_train, limelight, distance, speed)

def drive_sideways_command(drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain,
                          limelight: Limelight,
                          offset: float):
    distance = offset
    speed = 0.5
    return DriveSidewaysCommand(drive_train, limelight, distance, speed)

def drive_backward_command(drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain,
                           limelight: Limelight):
    distance = wpimath.units.inchesToMeters(-11.0)
    speed = -0.5
    return DriveStraightCommand(drive_train, limelight, distance, speed)