import pathplannerlib.path
import pathplannerlib.auto
import wpimath.units
import commands2
import subsystems.command_swerve_drivetrain
import subsystems.elevator
import subsystems.limelight
from wpimath.geometry import Rotation2d, Pose2d

import subsystems.mock_drivetrain

# Positions for the robot to line up to the April Tags, indexed by April Tag IDs
positions = {}
positions[1] = Pose2d(16.40, 1.02, Rotation2d.fromDegrees(130)) # Red Coral Pickup Left
positions[2] = Pose2d(16.41, 7.00, Rotation2d.fromDegrees(50)) # Red Coral Pickup Right
positions[3] = Pose2d(11.48, 7.55, Rotation2d.fromDegrees(90)) # Red side, Blue's Algae
positions[6] = Pose2d(13.73, 2.87, Rotation2d.fromDegrees(120)) # Red Coral
positions[7] = Pose2d(14.44, 4.02, Rotation2d.fromDegrees(180)) # Red Coral
positions[8] = Pose2d(13.77, 5.21, Rotation2d.fromDegrees(-120)) # Red Coral
positions[9] = Pose2d(12.31, 5.22, Rotation2d.fromDegrees(-60)) # Red Coral
positions[10] = Pose2d(11.70, 4.00, Rotation2d.fromDegrees(0)) # Red Coral
positions[11] = Pose2d(12.41, 2.81, Rotation2d.fromDegrees(60)) # Red Coral
positions[12] = Pose2d(1.18, 1.08, Rotation2d.fromDegrees(-127)) # Blue Coral Pickup Right
positions[13] = Pose2d(1.13, 6.94, Rotation2d.fromDegrees(128)) # Blue Coral Pickup Left
positions[16] = Pose2d(6.02, 0.52, Rotation2d.fromDegrees(-90)) # Blue Coral
positions[17] = Pose2d(3.84, 2.81, Rotation2d.fromDegrees(60)) # Blue Coral 
positions[18] = Pose2d(3.04, 4.01, Rotation2d.fromDegrees(0)) # Blue Coral
positions[19] = Pose2d(3.78, 5.24, Rotation2d.fromDegrees(-60)) # Blue Coral
positions[20] = Pose2d(5.18, 5.19, Rotation2d.fromDegrees(-120)) # Blue Coral
positions[21] = Pose2d(5.85, 4.03, Rotation2d.fromDegrees(180)) # Blue Coral
positions[22] = Pose2d(5.22, 2.83, Rotation2d.fromDegrees(120)) # Blue Coral

def pathplanner_constraints(): 
    # Create the constraints to use while pathfinding
    return pathplannerlib.path.PathConstraints(
        3.0,
        3.0,
        wpimath.units.degreesToRadians(540),
        wpimath.units.degreesToRadians(720)
    )

def get_auto_command(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     ) -> commands2.Command:
    
    # Assume we start on the black line, but ultimately we don't care where we start.
        
    # Place first coral
    cmds = commands2.SequentialCommandGroup()
    cmds.addCommands(drive_to_pose(11))
    cmds.addCommands(place_coral(drivetrain, front_limelight, elevator, 50, 50, False))

    # Go pick up second coral
    cmds.addCommands(drive_to_pose(1))
    cmds.addCommands(pickup_coral(drivetrain, back_limelight))

    # Place second coral
    cmds.addCommands(drive_to_pose(6))
    cmds.addCommands(place_coral(drivetrain, front_limelight, elevator, 50, 50, False))

    # Go pick up third coral
    cmds.addCommands(drive_to_pose(1))
    cmds.addCommands(pickup_coral(drivetrain, back_limelight))

    # Place third coral
    cmds.addCommands(drive_to_pose(6))
    cmds.addCommands(place_coral(drivetrain, front_limelight, elevator, 50, 50, True))

    return cmds

def drive_to_pose(pose_id: int):
    return pathplannerlib.auto.AutoBuilder.pathfindToPose(
            positions[pose_id],
            pathplanner_constraints(),
            0.0
        )

def place_coral(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                limelight: subsystems.limelight.Limelight,
                elevator: subsystems.elevator.Elevator,
                elevator_position: float,
                wrist_position: float,
                right_post: bool, # right or left coral post
                ):
    cmds = commands2.SequentialCommandGroup()

    # Line up according to the limelight AprilTag data.
    alignCoral = subsystems.limelight.line_up_left_post_command(drivetrain, limelight)
    if right_post:
        alignCoral = subsystems.limelight.line_up_right_post_command(drivetrain, limelight)
    cmds.addCommands(alignCoral)

    # Move forward blindly to fill the gap where we lose sight of the april tag.
    driveForward = subsystems.limelight.drive_forward_command(drivetrain, limelight)
    # Move elevator up to position
    moveElevatorToCoralPosition = subsystems.elevator.MoveElevatorToPosition(elevator, elevator_position)
    # Move the wrist up to position
    moveWrist = subsystems.elevator.coralWristToPosition(elevator, wrist_position)
    # Do those things in parallel (while driving forward, move the stuff into position)
    parallelGroupUp = driveForward.alongWith(moveElevatorToCoralPosition).alongWith(moveWrist)
    cmds.addCommands(parallelGroupUp)

    # Shoot the coral out onto the post. (TODO add trigger for isFinished)
    shootCoral = subsystems.elevator.CoralOutCommand(elevator, 1.0)
    cmds.addCommands(commands2.button.Trigger(lambda: shootCoral.timer.hasElapsed(1)).whileFalse(shootCoral))

    # Move the wrist back down to origin.
    wristDown = subsystems.elevator.coralWristToPosition(elevator, 0)
    # Move the elevator back down to origin.
    moveElevatorToFloor = subsystems.elevator.MoveElevatorToPosition(elevator, 0)
    # Do those in parallel while backing up slightly.
    driveBackward = subsystems.limelight.drive_backward_command(drivetrain, limelight)
    parallelGroupdown = wristDown.alongWith(moveElevatorToFloor).alongWith(driveBackward)
    cmds.addCommands(parallelGroupdown)

    return cmds

def pickup_coral(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain,
                 limelight: subsystems.limelight.Limelight):
    
    cmds = commands2.SequentialCommandGroup()

    # Line up according to the limelight AprilTag data.
    alignAprilTag = subsystems.limelight.line_up_for_coral_pickup(drivetrain, limelight)
    cmds.addCommands(alignAprilTag)

    # Move forward blindly
    driveForward = subsystems.limelight.drive_forward_command(drivetrain, limelight)
    cmds.addCommands(driveForward)

    # Wait until we receive a coral
    # TODO

    return cmds