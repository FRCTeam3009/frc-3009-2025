import commands2
import wpimath.geometry
import subsystems.command_swerve_drivetrain
import wpimath
import wpimath.units
import phoenix6.swerve

FORWARD_OFFSET = wpimath.units.inchesToMeters(11.0) # inches away from the 
CORAL_POST_OFFSET = wpimath.units.inchesToMeters(6.47) # inches offset from center of AprilTag

ROBOT_RELATIVE = (
            phoenix6.swerve.requests.RobotCentric()
            .with_drive_request_type(
                phoenix6.swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )

class DriveRobotRelativeCommand(commands2.Command):
    def __init__(self, 
                 drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                 pose: wpimath.geometry.Pose2d,
                 speed: float,
                 ):
        self.addRequirements(drive_train)
        self.drive_train = drive_train
        self.pose = pose
        self.speed = speed

        self.start_pose = wpimath.geometry.Pose2d()
        self.end_pose = wpimath.geometry.Pose2d()

    def initialize(self):
        self.start_pose = self.drive_train.get_state().pose
        x = self.start_pose.X() + self.pose.X()
        y = self.start_pose.Y() + self.pose.Y()
        r = self.start_pose.rotation().radians() + self.pose.rotation().radians()
        self.end_pose = wpimath.geometry.Pose2d(x, y, r)

    def execute(self):
        forward = 0.0
        horizontal = 0.0
        rotation = 0.0

        x = self.pose.X()
        y = self.pose.Y()
        r = self.pose.rotation().degrees()

        if x > 0:
            forward = self.speed
        elif x < 0:
            forward = -1 * self.speed
        if y > 0:
            horizontal = self.speed
        elif y < 0:
            horizontal = -1 * self.speed
        if r < 5:
            rotation = self.speed
        elif r > 5:
            rotation = -1 * self.speed

        drive_request = lambda: ROBOT_RELATIVE.with_velocity_x(forward).with_velocity_y(horizontal).with_rotational_rate(rotation)
        self.drive_train.apply_request(drive_request).schedule()

    def isFinished(self):
        pose = self.drive_train.get_state().pose
        x = self.end_pose.X() - pose.X()
        y = self.end_pose.Y() - pose.Y()
        r = self.end_pose.rotation().degrees() - pose.rotation().degrees()
        inches = wpimath.units.inchesToMeters(1)
        return abs(x) < 2 * inches and abs(y) < 2 * inches and abs(r) < 10

def drive_forward_command(drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, offset: float, speed: float):
    pose = wpimath.geometry.Pose2d(offset, 0.0, 0.0)
    return DriveRobotRelativeCommand(drive_train, pose, speed)

def drive_sideways_command(drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, offset: float, speed: float):
    pose = wpimath.geometry.Pose2d(0.0, offset, 0.0)
    return DriveRobotRelativeCommand(drive_train, pose, speed)

def drive_backward_command(drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, offset: float, speed: float):
    pose = wpimath.geometry.Pose2d(-1 * offset, 0.0, 0.0)
    return DriveRobotRelativeCommand(drive_train, pose, -1 * speed)
