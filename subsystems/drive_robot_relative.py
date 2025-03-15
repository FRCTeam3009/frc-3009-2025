import commands2
import wpimath.geometry
import subsystems.command_swerve_drivetrain
import wpimath
import wpimath.units
import phoenix6.swerve

FORWARD_OFFSET = wpimath.units.inchesToMeters(22.0) # inches away from the Coral posts
CORAL_POST_OFFSET = wpimath.units.inchesToMeters(-3.0) # inches offset from center of AprilTag
ONE_INCH = wpimath.units.inchesToMeters(1)
TWO_INCHES = wpimath.units.inchesToMeters(2)

ROBOT_RELATIVE = (
            phoenix6.swerve.requests.RobotCentric()
            .with_drive_request_type(
                phoenix6.swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )

TURBO_SPEED = 1.0
NORMAL_SPEED = 0.5
SLOW_SPEED = 0.25

class DriveRobotRelativeCommand(commands2.Command):
    def __init__(self, 
                 drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                 offset: wpimath.geometry.Transform2d,
                 speed: float,
                 ):
        self.addRequirements(drive_train)
        self.drive_train = drive_train
        self.offset = offset
        self.speed = speed

        self.forward = 0.0
        self.horizontal = 0.0
        self.rotation = 0.0

        self.start_pose : wpimath.geometry.Pose2d
        self.start_pose = None

        self.end_pose : wpimath.geometry.Pose2d
        self.end_pose = None


    def initialize(self):
        self.start_pose = self.drive_train.get_state_copy().pose
        self.end_pose = self.start_pose + self.offset
        
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

        self.forward = forward
        self.horizontal = horizontal
        self.rotation = rotation
        
    def execute(self):
        current_pose = self.drive_train.get_state_copy().pose
        diff = self.end_pose - current_pose

        if abs(diff.X()) < ONE_INCH:
            self.forward = 0.0
        if abs(diff.Y()) < ONE_INCH:
            self.horizontal = 0.0
        if abs(diff.rotation().degrees()) < 5:
            self.rotation = 0.0
        
        drive_request = lambda: ROBOT_RELATIVE.with_velocity_x(self.forward).with_velocity_y(self.horizontal).with_rotational_rate(self.rotation)
        self.drive_cmd = self.drive_train.apply_request(drive_request)
        self.drive_cmd.execute()

    def isFinished(self):
        current_pose = self.drive_train.get_state_copy().pose
        diff = self.end_pose - current_pose

        return abs(diff.X()) < ONE_INCH and abs(diff.Y()) < ONE_INCH and abs(diff.rotation().degrees()) < 5
    
    def end(self, interrupted):
        drive_request = lambda: ROBOT_RELATIVE.with_velocity_x(0.0).with_velocity_y(0.0).with_rotational_rate(0.0)
        self.drive_train.apply_request(drive_request).execute()

def drive_forward_command(drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, offset: float, speed: float):
    pose = wpimath.geometry.Transform2d(offset, 0.0, 0.0)
    return DriveRobotRelativeCommand(drive_train, pose, speed)

def drive_sideways_command(drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, offset: float, speed: float):
    pose = wpimath.geometry.Transform2d(0.0, offset, 0.0)
    return DriveRobotRelativeCommand(drive_train, pose, speed)

def drive_backward_command(drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, offset: float, speed: float):
    pose = wpimath.geometry.Transform2d(-1 * offset, 0.0, 0.0)
    return DriveRobotRelativeCommand(drive_train, pose, speed)
