import commands2
import wpimath.geometry
import subsystems.command_swerve_drivetrain
import wpimath
import wpimath.units
import phoenix6.swerve

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

        self.end_pose : wpimath.geometry.Pose2d
        self.end_pose = None

    def initialize(self):
        self.start_pose = self.drive_train.get_state_copy().pose
        x = self.start_pose.X() + self.offset.X()
        y = self.start_pose.Y() + self.offset.Y()
        r = self.start_pose.rotation().radians() + self.offset.rotation().radians()
        self.end_pose = wpimath.geometry.Pose2d(x, y, r)

    def execute(self):
        forward = 0.0
        horizontal = 0.0
        rotation = 0.0

        current_pose = self.drive_train.get_state_copy().pose

        # TODO debugging why it overshoots when trying to do robot relative drive.
        print("PENGUINS - " + str(self.end_pose.Y()) + " " + str(current_pose.Y()))

        x = self.end_pose.X() - current_pose.X()
        y = self.end_pose.Y() - current_pose.Y()
        r = self.end_pose.rotation().degrees() - current_pose.rotation().degrees()

        if x > ONE_INCH:
            forward = self.speed
        elif x < -ONE_INCH:
            forward = -1 * self.speed
        if y > ONE_INCH:
            horizontal = self.speed
        elif y < -ONE_INCH:
            horizontal = -1 * self.speed
        if r > 5:
            rotation = self.speed
        elif r < 5:
            rotation = -1 * self.speed

        rotation = 0.0

        drive_request = lambda: ROBOT_RELATIVE.with_velocity_x(forward).with_velocity_y(horizontal).with_rotational_rate(rotation)
        self.drive_train.apply_request(drive_request).execute()

    def isFinished(self):
        current_pose = self.drive_train.get_state_copy().pose
        x = self.end_pose.X() - current_pose.X()
        y = self.end_pose.Y() - current_pose.Y()
        r = self.end_pose.rotation().degrees() - current_pose.rotation().degrees()
        print("PENGUINS - " + str(x) + " " + str(y) + " " + str(r) + " " + str(TWO_INCHES))
        return abs(x) < TWO_INCHES and abs(y) < TWO_INCHES and abs(r) < 10
    
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
    return DriveRobotRelativeCommand(drive_train, pose, -1 * speed)
