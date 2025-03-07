import commands2
import wpimath.geometry
import subsystems.command_swerve_drivetrain
import wpimath
import wpimath.units
import phoenix6.swerve
import math

FORWARD_OFFSET = wpimath.units.inchesToMeters(11.0) # inches away from the 
CORAL_POST_OFFSET = wpimath.units.inchesToMeters(6.47) # inches offset from center of AprilTag
ONE_INCH = wpimath.units.inchesToMeters(1)
TWO_INCHES = wpimath.units.inchesToMeters(2)

ROBOT_RELATIVE = (
            phoenix6.swerve.requests.RobotCentric()
            .with_drive_request_type(
                phoenix6.swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )

class DriveRobotRelativeCommand(commands2.Command):
    def __init__(self, 
                 drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                 offset: wpimath.geometry.Pose2d,
                 speed: float,
                 ):
        self.addRequirements(drive_train)
        self.drive_train = drive_train
        self.offset = offset
        self.speed = speed

        self.start_pose : wpimath.geometry.Pose2d
        self.start_pose = None


    def initialize(self):
        self.start_pose = self.drive_train.get_state_copy().pose
        
        
        forward = 0.0
        horizontal = 0.0
        rotation = 0.0

        compare_x = self.offset.X()
        compare_y = self.offset.Y()
        compare_r = self.offset.rotation().degrees()

        if compare_x > ONE_INCH:
            forward = self.speed
        elif compare_x < -ONE_INCH:
            forward = -1 * self.speed
        if compare_y > ONE_INCH:
            horizontal = self.speed
        elif compare_y < -ONE_INCH:
            horizontal = -1 * self.speed
        if compare_r > 3:
            rotation = self.speed
        elif compare_r < -3:
            rotation = -1 * self.speed

        drive_request = lambda: ROBOT_RELATIVE.with_velocity_x(forward).with_velocity_y(horizontal).with_rotational_rate(rotation)
        self.drive_cmd = self.drive_train.apply_request(drive_request)
        
    def execute(self):
        self.drive_cmd.execute()

    def isFinished(self):
        current_pose = self.drive_train.get_state_copy().pose
        x = current_pose.X() - self.start_pose.X()
        y = current_pose.Y() - self.start_pose.Y()
        r = current_pose.rotation().degrees() - self.start_pose.rotation().degrees()

        compare_x = self.offset.X() - x
        compare_y = self.offset.Y() - y
        compare_r = self.offset.rotation().degrees() - r

        return abs(compare_x) < ONE_INCH and abs(compare_y) < ONE_INCH and abs(compare_r) < 5
    
    def end(self, interrupted):
        drive_request = lambda: ROBOT_RELATIVE.with_velocity_x(0.0).with_velocity_y(0.0).with_rotational_rate(0.0)
        self.drive_train.apply_request(drive_request).execute()

def drive_forward_command(drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, offset: float, speed: float):
    pose = wpimath.geometry.Pose2d(offset, 0.0, 0.0)
    return DriveRobotRelativeCommand(drive_train, pose, speed)

def drive_sideways_command(drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, offset: float, speed: float):
    pose = wpimath.geometry.Pose2d(0.0, offset, 0.0)
    return DriveRobotRelativeCommand(drive_train, pose, speed)

def drive_backward_command(drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, offset: float, speed: float):
    pose = wpimath.geometry.Pose2d(-1 * offset, 0.0, 0.0)
    return DriveRobotRelativeCommand(drive_train, pose, speed)
