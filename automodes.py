import pathplannerlib.path
import pathplannerlib.auto
import wpimath.geometry
import wpimath.units
import commands2
import subsystems.command_swerve_drivetrain
import subsystems.drive_robot_relative
import subsystems.elevator
import subsystems.limelight
import ntcore
import wpilib
import commands2
import subsystems.limelight_positions
import subsystems.shooter
import subsystems.wrist
from phoenix6 import swerve
from wpimath.geometry import Rotation2d, Pose2d
import subsystems.wrist

# Positions for the robot to line up to the April Tags, indexed by April Tag IDs
positions = {}
positions[1] = Pose2d(16.44, 1.03, Rotation2d.fromDegrees(-52)) # Red Coral Pickup Left
positions[2] = Pose2d(16.45, 7.03, Rotation2d.fromDegrees(53)) # Red Coral Pickup Right
positions[3] = Pose2d(11.48, 7.55, Rotation2d.fromDegrees(90)) # Red side, Blue's Algae
positions[6] = Pose2d(14.13, 2.23, Rotation2d.fromDegrees(120)) # Red Coral
positions[7] = Pose2d(14.92, 4.04, Rotation2d.fromDegrees(180)) # Red Coral
positions[8] = Pose2d(14.00, 5.64, Rotation2d.fromDegrees(-120)) # Red Coral
positions[9] = Pose2d(12.16, 5.55, Rotation2d.fromDegrees(-60)) # Red Coral right
positions[10] = Pose2d(11.20, 4.00, Rotation2d.fromDegrees(0)) # Red Coral center
positions[11] = Pose2d(12.05, 2.14, Rotation2d.fromDegrees(60)) # Red Coral left
positions[12] = Pose2d(1.15, 0.99, Rotation2d.fromDegrees(-127)) # Blue Coral Pickup Right
positions[13] = Pose2d(1.17, 7.06, Rotation2d.fromDegrees(127)) # Blue Coral Pickup Left
positions[16] = Pose2d(6.02, 0.52, Rotation2d.fromDegrees(-90)) # Blue Coral
positions[17] = Pose2d(3.67, 2.25, Rotation2d.fromDegrees(60)) # Blue Coral 
positions[18] = Pose2d(2.56, 4.03, Rotation2d.fromDegrees(0)) # Blue Coral
positions[19] = Pose2d(3.41, 5.59, Rotation2d.fromDegrees(-60)) # Blue Coral
positions[20] = Pose2d(5.55, 5.73, Rotation2d.fromDegrees(-120)) # Blue Coral left
positions[21] = Pose2d(6.35, 3.99, Rotation2d.fromDegrees(180)) # Blue Coral center
positions[22] = Pose2d(5.68, 2.14, Rotation2d.fromDegrees(120)) # Blue Coral right

# closer
positions[23] = Pose2d(11.76, 4.21, Rotation2d.fromDegrees(0)) #red
positions[24] = Pose2d(5.79, 3.87, Rotation2d.fromDegrees(180)) #blue
positions[25] = Pose2d(5.00, 5.27, Rotation2d.fromDegrees(-120)) #blueleft
positions[26] = Pose2d(5.00, 2.85, Rotation2d.fromDegrees(120)) #blue right
positions[27] = Pose2d(12.55, 2.82, Rotation2d.fromDegrees(60)) #red left
positions[28] = Pose2d(12.57, 5.24, Rotation2d.fromDegrees(-60)) #red right



class AutoCommand():
    drive_to_pose = 1
    drive_pickup = 2
    drive_place = 3
    wait = 4
    def __init__(self, position : Pose2d, type : int, elevator_pose : float, wrist_pose : float, aprtag: int):
        self.position = position
        self.type = type
        self.elevator_pose = elevator_pose
        self.wrist_pose = wrist_pose
        self.april_tag_id = aprtag


def pathplanner_constraints(): 
    # Create the constraints to use while pathfinding
    return pathplannerlib.path.PathConstraints(
        2.0,
        2.0,
        wpimath.units.rotationsToRadians(0.75),
        wpimath.units.rotationsToRadians(0.75), 
    )

def noob_auto_drive_straight_forward(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter,
                     ) -> commands2.Command:
    
    return subsystems.drive_robot_relative.drive_forward_command(drivetrain, wpimath.units.meters(1.5), subsystems.drive_robot_relative.NORMAL_SPEED)


def forward_blue(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     _: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(8, 4, Rotation2d.fromDegrees(180))
    drivetrain.reset_pose(approx_start)
    cmd = AutoCommand(positions[21], AutoCommand.drive_place, subsystems.elevator.MoveElevatorToPosition.L1, subsystems.wrist.CoralWristToPosition.L1, 21)
    return schedule_coral_place(cmd, drivetrain, front_limelight, elevator, wrist, shooter)

def forward_left_blue(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     _: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(8, 6, Rotation2d.fromDegrees(180))
    drivetrain.reset_pose(approx_start)
    cmd = AutoCommand(positions[20], AutoCommand.drive_place, subsystems.elevator.MoveElevatorToPosition.L1, subsystems.wrist.CoralWristToPosition.L1, 20)
    return schedule_coral_place(cmd, drivetrain, front_limelight, elevator, wrist, shooter)

def forward_right_blue(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     _: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(7.5, 2, Rotation2d.fromDegrees(180))
    drivetrain.reset_pose(approx_start)
    cmds = commands2.SequentialCommandGroup()
    cmd = AutoCommand(positions[22], AutoCommand.drive_place, subsystems.elevator.MoveElevatorToPosition.L1, subsystems.wrist.CoralWristToPosition.L1, 22)
    schedule_coral = schedule_coral_place(cmd, drivetrain, front_limelight, elevator, wrist, shooter)
    cmds.addCommands(WaitCommand(drivetrain))
    cmds.addCommands(schedule_coral)
    return cmds

def forward_red(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     _: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(10, 4, 0)
    drivetrain.reset_pose(approx_start)
    cmd = AutoCommand(positions[10], AutoCommand.drive_place, subsystems.elevator.MoveElevatorToPosition.L1, subsystems.wrist.CoralWristToPosition.L1, 10)
    return schedule_coral_place(cmd, drivetrain, front_limelight, elevator, wrist, shooter)

def forward_left_red(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     _: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(10, 2, 0)
    drivetrain.reset_pose(approx_start)
    cmd = AutoCommand(positions[11], AutoCommand.drive_place, subsystems.elevator.MoveElevatorToPosition.L1, subsystems.wrist.CoralWristToPosition.L1, 11)
    return schedule_coral_place(cmd, drivetrain, front_limelight, elevator, wrist, shooter)

def forward_right_red(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     _: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(10, 6, 0)
    drivetrain.reset_pose(approx_start)
    cmds = commands2.SequentialCommandGroup()
    cmd = AutoCommand(positions[9], AutoCommand.drive_place, subsystems.elevator.MoveElevatorToPosition.L1, subsystems.wrist.CoralWristToPosition.L1, 9)
    schedule_coral = schedule_coral_place(cmd, drivetrain, front_limelight, elevator, wrist, shooter)
    cmds.addCommands(WaitCommand(drivetrain))
    cmds.addCommands(schedule_coral)
    return cmds

def get_test_auto_place_coral(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter,
                     ) -> commands2.Command:
    cmd = AutoCommand(positions[1], AutoCommand.drive_place, subsystems.elevator.MoveElevatorToPosition.L1, subsystems.wrist.CoralWristToPosition.L1, 19)
    return place_coral(
        cmd,
        drivetrain, 
        front_limelight,
        elevator,
        wrist,
        shooter,
        )

def get_test_auto_lineup_to_coral(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter,
                     ) -> commands2.Command:
    targetpose = front_limelight.target_poses[19].get_average_pose()
    targetpose = subsystems.limelight_positions.correct_target_pose(targetpose)
    offset = wpimath.geometry.Transform2d(targetpose.X(), targetpose.Y(), targetpose.rotation())

    return subsystems.drive_robot_relative.DriveRobotRelativeCommand(drivetrain, offset, subsystems.drive_robot_relative.SLOW_SPEED)

def get_test_auto_drive_forward_coral(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    return subsystems.drive_robot_relative.drive_forward_command(drivetrain, subsystems.drive_robot_relative.FORWARD_OFFSET, subsystems.drive_robot_relative.SLOW_SPEED)


def get_test_auto_drive_sideways_coral(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    return subsystems.drive_robot_relative.drive_sideways_command(drivetrain, subsystems.drive_robot_relative.CORAL_POST_OFFSET, subsystems.drive_robot_relative.SLOW_SPEED)

def get_test_auto_drive_backward_pickup(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter,
                     ) -> commands2.Command:
    return subsystems.drive_robot_relative.drive_backward_command(drivetrain, subsystems.drive_robot_relative.FORWARD_OFFSET, subsystems.drive_robot_relative.SLOW_SPEED)

def get_test_auto_elevator_position(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter,
                     ) -> commands2.Command:
    return subsystems.elevator.MoveElevatorToPosition(elevator, subsystems.elevator.MoveElevatorToPosition.L1)

def get_test_auto_wrist_position(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter,
                     ) -> commands2.Command:
    return subsystems.wrist.CoralWristToPosition(wrist, subsystems.wrist.CoralWristToPosition.L1)

def get_test_auto_offset_apriltag(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter,
                     ) -> commands2.Command:
    return subsystems.limelight.lineupCommand(drivetrain, front_limelight, 19).withTimeout(3.0)


def get_test_auto_lineup_and_place(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter,
                     ) -> commands2.Command:
    cmds = commands2.SequentialCommandGroup()
    cmds.addCommands(subsystems.elevator.MoveElevatorToPosition(elevator, subsystems.elevator.MoveElevatorToPosition.auto_pose))
    cmds.addCommands(WaitCommand(drivetrain).withTimeout(1.0))
    auto_command = AutoCommand(positions[19], AutoCommand.wait, subsystems.elevator.MoveElevatorToPosition.L1, subsystems.wrist.CoralWristToPosition.L1, 19)   
    cmds.addCommands(place_coral(auto_command, drivetrain, front_limelight, elevator, wrist, shooter))
    return cmds

def get_test_auto_fake_apriltag(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter,
                     ) -> commands2.Command:
    x = wpimath.units.inchesToMeters(12)
    y = wpimath.units.inchesToMeters(5)
    r = wpimath.units.degreesToRadians(15)
    targetpose = Pose2d(x, y, r)
    offset = wpimath.geometry.Transform2d(targetpose.X(), targetpose.Y(), targetpose.rotation())

    return subsystems.drive_robot_relative.DriveRobotRelativeCommand(drivetrain, offset, subsystems.drive_robot_relative.NORMAL_SPEED)

# left
def get_red_auto_1(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter,
                     ) -> commands2.Command:
    approx_start = Pose2d(10, 2, 0)
    auto_commands = [AutoCommand(positions[11], AutoCommand.drive_place, subsystems.elevator.MoveElevatorToPosition.L1, subsystems.wrist.CoralWristToPosition.L1, 11),
                     ]
    return get_auto_command(drivetrain, front_limelight, back_limelight, elevator, wrist, approx_start, auto_commands, shooter)

# right
def get_red_auto_2(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(10, 6, 0)
    auto_commands = [AutoCommand(positions[9], AutoCommand.drive_place, subsystems.elevator.MoveElevatorToPosition.L1, subsystems.wrist.CoralWristToPosition.L1, 9),
                     ]
    return get_auto_command(drivetrain, front_limelight, back_limelight, elevator, wrist, approx_start, auto_commands, shooter)

# left
def get_blue_auto_1(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(7.5, 6, 3.14)
    auto_commands = [AutoCommand(positions[20], AutoCommand.drive_place, subsystems.elevator.MoveElevatorToPosition.L1, subsystems.wrist.CoralWristToPosition.L1, 20),
                     ]
    return get_auto_command(drivetrain, front_limelight, back_limelight, elevator, wrist, approx_start, auto_commands, shooter)

# right
def get_blue_auto_2(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(7.5, 2, 3.14)
    auto_commands = [AutoCommand(positions[22], AutoCommand.drive_place, subsystems.elevator.MoveElevatorToPosition.L1, subsystems.wrist.CoralWristToPosition.L1, 22),
                     ]
    return get_auto_command(drivetrain, front_limelight, back_limelight, elevator, wrist, approx_start, auto_commands, shooter)

def center_red(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(10, 4, 0)
    auto_commands = [AutoCommand(positions[10], AutoCommand.drive_place, subsystems.elevator.MoveElevatorToPosition.L1, subsystems.wrist.CoralWristToPosition.L1, 10),
                     AutoCommand(positions[1], AutoCommand.wait, subsystems.elevator.MoveElevatorToPosition.L1, subsystems.wrist.CoralWristToPosition.L1, 10),
                    ]
    return get_auto_command(drivetrain, front_limelight, back_limelight, elevator, wrist, approx_start, auto_commands, shooter)

def center_blue(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(7.5, 4, 3.14)
    auto_commands = [AutoCommand(positions[21], AutoCommand.drive_place, subsystems.elevator.MoveElevatorToPosition.L1, subsystems.wrist.CoralWristToPosition.L1, 21), 
                     AutoCommand(positions[1], AutoCommand.wait, subsystems.elevator.MoveElevatorToPosition.L1, subsystems.wrist.CoralWristToPosition.L1, 21),
                    ]
    return get_auto_command(drivetrain, front_limelight, back_limelight, elevator, wrist, approx_start, auto_commands, shooter)


def get_auto_command(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     approx_start: Pose2d,
                     auto_commands: list[AutoCommand],
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    
    # Assume we start on the black line, but ultimately we don't care where we start.
    
    drivetrain.reset_pose(approx_start)

    cmds = commands2.SequentialCommandGroup()

    for command in auto_commands:
        if command.type == AutoCommand.wait:
            cmds.addCommands(WaitCommand(drivetrain))
        elif command.type == AutoCommand.drive_pickup:
            drive_pickup = schedule_drive_pickup(command, back_limelight, elevator, wrist)
            cmds.addCommands(drive_pickup)
        elif command.type == AutoCommand.drive_place:
            coral_place = schedule_coral_place(command, drivetrain, front_limelight, elevator, wrist, shooter)
            cmds.addCommands(coral_place)
        elif command.type == AutoCommand.drive_to_pose:
            cmds.addCommands(drive_to_pose(command))
    return cmds

def drive_to_pose(cmd : AutoCommand):
    return pathplannerlib.auto.AutoBuilder.pathfindToPose(
            cmd.position,
            pathplanner_constraints(),
            0.0,
        )

def place_coral(cmd: AutoCommand,
                drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain,
                limelight: subsystems.limelight.Limelight,
                elevator: subsystems.elevator.Elevator,
                wrist: subsystems.wrist.Wrist,
                shooter: subsystems.shooter.Shooter
                ):
    cmds = commands2.SequentialCommandGroup()

    # Move the wrist up to position
    moveWrist = subsystems.wrist.CoralWristToPosition(wrist, cmd.wrist_pose).withTimeout(2.0)
    cmds.addCommands(moveWrist)

    # Move elevator up to position
    moveElevatorToCoralPosition = subsystems.elevator.MoveElevatorToPosition(elevator, cmd.elevator_pose).withTimeout(3.0)

     # Line up to april tag
    aprilTag = subsystems.limelight.lineupCommand(drivetrain, limelight, cmd.april_tag_id).withTimeout(3.0)
    cmds.addCommands(aprilTag.alongWith(moveElevatorToCoralPosition))

    # Shoot the coral out onto the post
    shootCoral = subsystems.shooter.CoralOutCommand(shooter, lambda: -subsystems.shooter.AUTO_SPEED).withTimeout(2.0)
    cmds.addCommands(shootCoral)

     # drive backwards
    driveBackwards = subsystems.drive_robot_relative.drive_backward_command(drivetrain, wpimath.units.inchesToMeters(12), subsystems.drive_robot_relative.SLOW_SPEED).withTimeout(3.0)
    cmds.addCommands(driveBackwards)
   

    return cmds

def pickup_coral(limelight: subsystems.limelight.Limelight,
                 elevator: subsystems.elevator.Elevator,
                 wrist: subsystems.wrist.Wrist):
    
    cmds = commands2.SequentialCommandGroup()

    # Line up according to the limelight AprilTag data.
    #alignAprilTag = subsystems.limelight.lineup_apriltag_command(drivetrain, limelight).withTimeout(2)
    #cmds.addCommands(alignAprilTag)

    # Move the elevator into position
    moveElevator = subsystems.elevator.MoveElevatorToPosition(elevator, subsystems.elevator.MoveElevatorToPosition.pickup).withTimeout(1.0)
    # Move the wrist into position
    moveWrist = subsystems.wrist.CoralWristToPosition(wrist, subsystems.wrist.CoralWristToPosition.pickup).withTimeout(1.0)
    cmds.addCommands(moveElevator.alongWith(moveWrist))

    # Wait until we receive a coral
    wait = subsystems.wrist.CoralWait(wrist.coral_sensor_receive).withTimeout(3)
    cmds.addCommands(wait)

    return cmds

class AutoDashboard():
    auto_map = {
        "noob_forward": noob_auto_drive_straight_forward,
        # "use_blue": forward_blue,
        # "use_red": forward_red,
        # "use_left_red": forward_left_red,
        # "use_right_red": forward_right_red,
        # "use_left_blue": forward_left_blue,
        # "use_right_blue": forward_right_blue,
        "redleft": get_red_auto_1,
        "redright": get_red_auto_2,
        "blueleft": get_blue_auto_1,
        "blueright": get_blue_auto_2,
        "centerred": center_red,
        "centerblue": center_blue,
        "test_backward_pickup": get_test_auto_drive_backward_pickup,
        "test_forward_coral": get_test_auto_drive_forward_coral,
        "test_sideways_coral": get_test_auto_drive_sideways_coral,
        "test_elevator_position": get_test_auto_elevator_position,
        "test_wrist_position": get_test_auto_wrist_position,
        "test_place_coral": get_test_auto_place_coral,
        "test_lineup_coral": get_test_auto_lineup_to_coral,
        "test_offset_apriltag": get_test_auto_offset_apriltag,
        "test_fake_apriltag": get_test_auto_fake_apriltag,
        "test_line_up_and_place": get_test_auto_lineup_and_place,
       }
    def __init__(self):
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self.table = self.nt_instance.getTable("auto modes")
        self.optionstopic = self.table.getStringArrayTopic("options")
        self.options_publisher = self.optionstopic.publish()
        self.options_publisher.set(list(self.auto_map.keys()))

        self.selectedtopic = self.table.getStringTopic("selected")
        self.selected_publisher = self.selectedtopic.publish()
        self.selected_publisher.set("noob_forward")
        self.selected_subscriber = self.selectedtopic.subscribe("noob_forward")
        self.current_auto = self.selected_subscriber.get()        

    def update(self):
        self.options_publisher.set(list(self.auto_map.keys()))
        self.current_auto = self.selected_subscriber.get()

    def get_current_auto_builder(self, drivetrain, front_limelight, back_limelight, elevator, wrist, shooter):
        auto_builder = self.auto_map[self.current_auto]
        return auto_builder(drivetrain, front_limelight, back_limelight, elevator, wrist, shooter)
    

class WaitCommand(commands2.Command):
    def __init__(self, drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain):
        self.drive_robot_relative = (
            swerve.requests.RobotCentric()
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )
        self.drivetrain = drivetrain
        self.timer = wpilib.Timer()
        
    def execute(self):
        drive_request = lambda: self.drive_robot_relative.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(0)
        self.drivetrain.apply_request(drive_request).execute()
        self.timer.start()

    def isFinished(self):
        if self.timer.hasElapsed(9):
            return True
        return False
    
def schedule_drive_pickup(cmd : AutoCommand,
                          limelight : subsystems.limelight.Limelight, 
                          elevator : subsystems.elevator.Elevator, 
                          wrist : subsystems.wrist.Wrist):
    cmds = commands2.SequentialCommandGroup()
    cmds.addCommands(drive_to_pose(cmd))
    cmds.addCommands(pickup_coral(limelight, elevator, wrist))
    return cmds

def schedule_coral_place(cmd : AutoCommand,
                         drivetrain : subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                         front_limelight : subsystems.limelight.Limelight, 
                         elevator : subsystems.elevator.Elevator, 
                         wrist : subsystems.wrist.Wrist, 
                         shooter: subsystems.shooter.Shooter):
    cmds = commands2.SequentialCommandGroup()
    cmds.addCommands(subsystems.elevator.MoveElevatorToPosition(elevator, subsystems.elevator.MoveElevatorToPosition.auto_pose).withTimeout(2.0))
    cmds.addCommands(WaitCommand(drivetrain).withTimeout(2.0)) # Wait to get April Tag position.
    cmds.addCommands(drive_to_pose(cmd))
    cmds.addCommands(place_coral(
        cmd,
        drivetrain,
        front_limelight, 
        elevator, 
        wrist, 
        shooter))
    return cmds