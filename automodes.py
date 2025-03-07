import pathplannerlib.path
import pathplannerlib.auto
import wpimath.units
import commands2
import subsystems.command_swerve_drivetrain
import subsystems.drive_robot_relative
import subsystems.elevator
import subsystems.limelight
import ntcore
import wpilib
import commands2
import subsystems.shooter
import subsystems.wrist
from phoenix6 import swerve
from wpimath.geometry import Rotation2d, Pose2d

import subsystems.mock_drivetrain
import subsystems.wrist

TRANSITION_SPEED = 0.0

# Positions for the robot to line up to the April Tags, indexed by April Tag IDs
positions = {}
positions[1] = Pose2d(16.40, 1.02, Rotation2d.fromDegrees(127)) # Red Coral Pickup Left
positions[2] = Pose2d(16.41, 7.00, Rotation2d.fromDegrees(-127)) # Red Coral Pickup Right
positions[3] = Pose2d(11.48, 7.55, Rotation2d.fromDegrees(90)) # Red side, Blue's Algae
positions[6] = Pose2d(14.13, 2.23, Rotation2d.fromDegrees(120)) # Red Coral
positions[7] = Pose2d(14.92, 4.04, Rotation2d.fromDegrees(180)) # Red Coral
positions[8] = Pose2d(14.00, 5.64, Rotation2d.fromDegrees(-120)) # Red Coral
positions[9] = Pose2d(12.16, 5.55, Rotation2d.fromDegrees(-60)) # Red Coral
positions[10] = Pose2d(11.20, 4.00, Rotation2d.fromDegrees(0)) # Red Coral
positions[11] = Pose2d(12.05, 2.14, Rotation2d.fromDegrees(60)) # Red Coral
positions[12] = Pose2d(1.18, 1.08, Rotation2d.fromDegrees(-307)) # Blue Coral Pickup Right
positions[13] = Pose2d(1.13, 6.94, Rotation2d.fromDegrees(307)) # Blue Coral Pickup Left
positions[16] = Pose2d(6.02, 0.52, Rotation2d.fromDegrees(-90)) # Blue Coral
positions[17] = Pose2d(3.67, 2.25, Rotation2d.fromDegrees(60)) # Blue Coral 
positions[18] = Pose2d(2.56, 4.03, Rotation2d.fromDegrees(0)) # Blue Coral
positions[19] = Pose2d(3.41, 5.59, Rotation2d.fromDegrees(-60)) # Blue Coral
positions[20] = Pose2d(5.55, 5.73, Rotation2d.fromDegrees(-120)) # Blue Coral
positions[21] = Pose2d(6.35, 3.99, Rotation2d.fromDegrees(180)) # Blue Coral
positions[22] = Pose2d(5.68, 2.14, Rotation2d.fromDegrees(120)) # Blue Coral

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
    def __init__(self, position : Pose2d, type : int, transition_speed : float):
        self.position = position
        self.type = type
        self.transition_speed = transition_speed


def pathplanner_constraints(): 
    # Create the constraints to use while pathfinding
    return pathplannerlib.path.PathConstraints(
        4.0,
        4.0,
        wpimath.units.rotationsToRadians(0.75),
        wpimath.units.rotationsToRadians(0.75), 
    )

def noob_auto_drive_straight_forward(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     ) -> commands2.Command:
    
    return subsystems.drive_robot_relative.drive_forward_command(drivetrain, wpimath.units.meters(1.5), 0.25)


def forward_blue(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(8, 4, Rotation2d.fromDegrees(180))
    drivetrain.reset_pose(approx_start)
    coral_score = subsystems.elevator.MoveElevatorToPosition.bottom
    return schedule_coral_place(drivetrain, front_limelight, elevator, wrist, positions[24], shooter, coral_score)

def forward_left_blue(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(8, 6, Rotation2d.fromDegrees(180))
    drivetrain.reset_pose(approx_start)
    return schedule_coral_place(drivetrain, front_limelight, elevator, wrist, positions[25], shooter)

def forward_right_blue(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(7.5, 2, Rotation2d.fromDegrees(180))
    drivetrain.reset_pose(approx_start)
    cmds = commands2.SequentialCommandGroup()
    coral_score = subsystems.elevator.MoveElevatorToPosition.bottom
    schedule_coral = schedule_coral_place(drivetrain, front_limelight, elevator, wrist, positions[26], shooter, coral_score)
    cmds.addCommands(WaitCommand(drivetrain))
    cmds.addCommands(schedule_coral)
    return cmds

def forward_red(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(10, 4, 0)
    drivetrain.reset_pose(approx_start)
    coral_score = subsystems.elevator.MoveElevatorToPosition.bottom
    return schedule_coral_place(drivetrain, front_limelight, elevator, wrist, positions[23], shooter, coral_score)

def forward_left_red(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(10, 2, 0)
    drivetrain.reset_pose(approx_start)
    return schedule_coral_place(drivetrain, front_limelight, elevator, wrist, positions[27], shooter)

def forward_right_red(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(10, 6, 0)
    drivetrain.reset_pose(approx_start)
    cmds = commands2.SequentialCommandGroup()
    coral_score = subsystems.elevator.MoveElevatorToPosition.bottom
    schedule_coral = schedule_coral_place(drivetrain, front_limelight, elevator, wrist, positions[28], coral_score, shooter)
    cmds.addCommands(WaitCommand(drivetrain))
    cmds.addCommands(schedule_coral)
    return cmds

def get_test_auto_place_coral(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     ) -> commands2.Command:
    return place_coral(
        drivetrain, 
        front_limelight, 
        elevator, 
        subsystems.elevator.MoveElevatorToPosition.top, 
        subsystems.wrist.CoralWristToPosition.top, 
        False)

def get_test_auto_lineup_to_coral(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     ) -> commands2.Command:
    return subsystems.limelight.line_up_command(drivetrain, front_limelight)

def get_test_auto_drive_forward_coral(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    return subsystems.drive_robot_relative.drive_forward_command(drivetrain, subsystems.drive_robot_relative.FORWARD_OFFSET, 0.25)


def get_test_auto_drive_sideways_coral(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    return subsystems.drive_robot_relative.drive_sideways_command(drivetrain, subsystems.drive_robot_relative.CORAL_POST_OFFSET, 0.25)

def get_test_auto_drive_backward_pickup(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter,
                     ) -> commands2.Command:
    return subsystems.drive_robot_relative.drive_backward_command(drivetrain, subsystems.drive_robot_relative.FORWARD_OFFSET, 0.25)

def get_test_auto_elevator_position(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     ) -> commands2.Command:
    return subsystems.elevator.MoveElevatorToPosition(elevator, subsystems.elevator.MoveElevatorToPosition.top)

def get_test_auto_wrist_position(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     ) -> commands2.Command:
    return subsystems.wrist.CoralWristToPosition(wrist, subsystems.wrist.CoralWristToPosition.top)

def get_test_auto_offset_apriltag(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     ) -> commands2.Command:
    return offset_april_tag(drivetrain, front_limelight)

def get_red_auto_1(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(10, 2, 0)
    auto_commands = [AutoCommand(positions[11], AutoCommand.drive_place, TRANSITION_SPEED),
                     AutoCommand(positions[1], AutoCommand.drive_pickup, TRANSITION_SPEED),
                     AutoCommand(positions[6], AutoCommand.drive_place, TRANSITION_SPEED),
                     ]
    return get_auto_command(drivetrain, front_limelight, back_limelight, elevator, wrist, approx_start, auto_commands, shooter)

def get_red_auto_2(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(10, 6, 0)
    auto_commands = [AutoCommand(positions[9], AutoCommand.drive_place, TRANSITION_SPEED),
                     AutoCommand(positions[2], AutoCommand.drive_pickup, TRANSITION_SPEED),
                     AutoCommand(positions[8], AutoCommand.drive_place, TRANSITION_SPEED),
                     ]
    return get_auto_command(drivetrain, front_limelight, back_limelight, elevator, wrist, approx_start, auto_commands, shooter)

def get_blue_auto_1(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(7.5, 6, 3.14)
    auto_commands = [AutoCommand(positions[20], AutoCommand.drive_place, TRANSITION_SPEED),
                     AutoCommand(positions[13], AutoCommand.drive_pickup, TRANSITION_SPEED),
                     AutoCommand(positions[19], AutoCommand.drive_place, TRANSITION_SPEED),
                     ]
    return get_auto_command(drivetrain, front_limelight, back_limelight, elevator, wrist, approx_start, auto_commands, shooter)

def get_blue_auto_2(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(7.5, 2, 3.14)
    auto_commands = [AutoCommand(positions[22], AutoCommand.drive_place, TRANSITION_SPEED),
                     AutoCommand(positions[12], AutoCommand.drive_pickup, TRANSITION_SPEED),
                     AutoCommand(positions[17], AutoCommand.drive_place, TRANSITION_SPEED),
                     ]
    return get_auto_command(drivetrain, front_limelight, back_limelight, elevator, wrist, approx_start, auto_commands, shooter)

def submit_red(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(10, 4, 0)
    auto_commands = [AutoCommand(positions[10], AutoCommand.drive_place, TRANSITION_SPEED),
                     AutoCommand(positions[3], AutoCommand.drive_to_pose, TRANSITION_SPEED),
                     AutoCommand(positions[1], AutoCommand.wait, TRANSITION_SPEED),
                     AutoCommand(positions[2], AutoCommand.drive_pickup, TRANSITION_SPEED),
                     AutoCommand(positions[9], AutoCommand.drive_place, TRANSITION_SPEED)
                    ]
    return get_auto_command(drivetrain, front_limelight, back_limelight, elevator, wrist, approx_start, auto_commands, shooter)

def submit_blue(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                     front_limelight: subsystems.limelight.Limelight, 
                     back_limelight: subsystems.limelight.Limelight,
                     elevator: subsystems.elevator.Elevator,
                     wrist: subsystems.wrist.Wrist,
                     shooter: subsystems.shooter.Shooter
                     ) -> commands2.Command:
    approx_start = Pose2d(7.5, 4, 3.14)
    pose = Pose2d(6.00, 1.50, Rotation2d.fromDegrees(90))
    auto_commands = [AutoCommand(positions[21], AutoCommand.drive_place, TRANSITION_SPEED), 
                    AutoCommand(pose, AutoCommand.drive_to_pose, TRANSITION_SPEED),
                    AutoCommand(positions[1], AutoCommand.wait, TRANSITION_SPEED),
                    AutoCommand(positions[12], AutoCommand.drive_pickup, TRANSITION_SPEED),
                    AutoCommand(positions[22], AutoCommand.drive_place, TRANSITION_SPEED)
                    ]
    return (drivetrain, front_limelight, back_limelight, elevator, wrist, approx_start, auto_commands, shooter)


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
            drive_pickup = schedule_drive_pickup(drivetrain, back_limelight, command.position, command.transition_speed, elevator, wrist)
            cmds.addCommands(drive_pickup)
        elif command.type == AutoCommand.drive_place:
            coral_place = schedule_coral_place(drivetrain, front_limelight, elevator, wrist, command.position, shooter)
            cmds.addCommands(coral_place)
        elif command.type == AutoCommand.drive_to_pose:
            cmds.addCommands(drive_to_pose(command.position, command.transition_speed))
    return cmds

def drive_to_pose(position: Pose2d, end_speed: float):
    return pathplannerlib.auto.AutoBuilder.pathfindToPose(
            position,
            pathplanner_constraints(),
            end_speed,
        )

def place_coral(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                limelight: subsystems.limelight.Limelight,
                elevator: subsystems.elevator.Elevator,
                elevator_position: float,
                wrist: subsystems.wrist.Wrist,
                wrist_position: float,
                right_post: bool, # right or left coral post
                shooter: subsystems.shooter.Shooter
                ):
    cmds = commands2.SequentialCommandGroup()

    # Move elevator up to position
    wrist_speed = 0.20
    moveElevatorToCoralPosition = subsystems.elevator.MoveElevatorToPosition(elevator, elevator_position).withTimeout(2.0)
    cmds.addCommands(moveElevatorToCoralPosition)
    # Move the wrist up to position
    moveWrist = subsystems.wrist.CoralWristToPosition(wrist, wrist_position, wrist_speed).withTimeout(1.0)
    cmds.addCommands(moveWrist)
    # Move forward blindly to fill the gap where we lose sight of the april tag.
    #driveForward = subsystems.drive_robot_relative.drive_forward_command(drivetrain, TODO).withTimeout(3.0)
    #cmds.addCommands(driveForward)

    # Shoot the coral out onto the post
    shootCoral = subsystems.shooter.CoralOutCommand(shooter, lambda: 1.0).withTimeout(1.0)
    cmds.addCommands(shootCoral)

    #driveBackward = subsystems.drive_robot_relative.drive_backward_command(drivetrain, TODO).withTimeout(3.0)
    #cmds.addCommands(driveBackward)
    # Move the elevator back down to origin.
    #moveElevatorToFloor = subsystems.elevator.MoveElevatorToPosition(elevator, 0).withTimeout(1.0)
    #moveElevatorBetter = subsystems.wrist.CoralWristToPosition(wrist, subsystems.wrist.CoralWristToPosition.pickup, wrist_speed).withTimeout(1.0).alongWith(moveElevatorToFloor)
    #cmds.addCommands(moveElevatorBetter)

    return cmds

def pickup_coral(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain,
                 limelight: subsystems.limelight.Limelight,
                 elevator: subsystems.elevator.Elevator,
                 wrist: subsystems.wrist.Wrist):
    
    cmds = commands2.SequentialCommandGroup()

    # Line up according to the limelight AprilTag data.
    #alignAprilTag = subsystems.limelight.line_up_command(drivetrain, limelight).withTimeout(2)
    #cmds.addCommands(alignAprilTag)

    # Move forward blindly
    driveBackward = subsystems.drive_robot_relative.drive_backward_command(drivetrain, subsystems.drive_robot_relative.FORWARD_OFFSET, 0.25).withTimeout(2)
    cmds.addCommands(driveBackward)

    # Wait until we receive a coral
    wait = subsystems.wrist.CoralWait(wrist.coral_sensor_receive).withTimeout(2)
    cmds.addCommands(wait)

    return cmds

class AutoDashboard():
    auto_map = {
        "noob_forward": noob_auto_drive_straight_forward,
        "use_blue": forward_blue,
        "use_red": forward_red,
        "use_left_red": forward_left_red,
        "use_right_red": forward_right_red,
        "use_left_blue": forward_left_blue,
        "use_right_blue": forward_right_blue,
        "redleft": get_red_auto_1,
        "redright": get_red_auto_2,
        "blueleft": get_blue_auto_1,
        "blueright": get_blue_auto_2,
        "hubrisred": submit_red,
        "hubrisblue": submit_blue,
        "test_backward_pickup": get_test_auto_drive_backward_pickup,
        "test_forward_coral": get_test_auto_drive_forward_coral,
        "test_sideways_coral": get_test_auto_drive_sideways_coral,
        "test_elevator_position": get_test_auto_elevator_position,
        "test_wrist_position": get_test_auto_wrist_position,
        "test_place_coral": get_test_auto_place_coral,
        "test_lineup_coral": get_test_auto_lineup_to_coral,
        "test_offset_apriltag": get_test_auto_offset_apriltag,
        
       }
    def __init__(self):
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self.table = self.nt_instance.getTable("auto modes")
        self.optionstopic = self.table.getStringArrayTopic("options")
        self.options_publisher = self.optionstopic.publish()
        self.options_publisher.set(list(self.auto_map.keys()))

        self.selectedtopic = self.table.getStringTopic("selected")
        self.selected_publisher = self.selectedtopic.publish()
        self.selected_publisher.set("use_right_blue")
        self.selected_subscriber = self.selectedtopic.subscribe("use_right_blue")
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
    
def schedule_drive_pickup(drivetrain : subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                          limelight : subsystems.limelight.Limelight, 
                          position : Pose2d, 
                          transition_speed : float, 
                          elevator : subsystems.elevator.Elevator, 
                          wrist : subsystems.wrist.Wrist):
    cmds = commands2.SequentialCommandGroup()
    cmds.addCommands(drive_to_pose(position, transition_speed))
    cmds.addCommands(pickup_coral(drivetrain, limelight, elevator, wrist))
    return cmds

def schedule_coral_place(drivetrain : subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                         front_limelight : subsystems.limelight.Limelight, 
                         elevator : subsystems.elevator.Elevator, 
                         wrist : subsystems.wrist.Wrist, 
                         position : Pose2d, 
                         shooter: subsystems.shooter.Shooter, 
                         coral_score = subsystems.elevator.MoveElevatorToPosition.middle):
    cmds = commands2.SequentialCommandGroup()
    cmds.addCommands(drive_to_pose(position, 0.0))
    #cmds.addCommands(offset_april_tag(drivetrain, front_limelight))
    cmds.addCommands(place_coral(
        drivetrain, 
        front_limelight, 
        elevator, 
        coral_score,
        wrist, 
        coral_score, 
        False,
        shooter))
    return cmds
    
def offset_april_tag(drivetrain: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain,
                     limelight: subsystems.limelight.Limelight,
                     ) -> commands2.Command:
    cmds = commands2.SequentialCommandGroup()

    # Line up according to the limelight AprilTag data.
    #alignAprilTag = subsystems.limelight.line_up_command(drivetrain, limelight).withTimeout(5.0)
    #cmds.addCommands(alignAprilTag)

    # drive sideways go line up with the coral post
    alignCoral = subsystems.drive_robot_relative.drive_sideways_command(drivetrain, subsystems.drive_robot_relative.CORAL_POST_OFFSET, 0.25).withTimeout(5.0)
    cmds.addCommands(alignCoral)

    return cmds