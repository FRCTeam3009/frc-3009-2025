#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.button
import commands2.cmd
from commands2.sysid import SysIdRoutine

from generated.tuner_constants import TunerConstants
from telemetry import Telemetry
import subsystems.elevator

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.path import PathConstraints
from phoenix6 import swerve
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.units import rotationsToRadians, degreesToRadians
import subsystems.limelight


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self._max_speed = (
            TunerConstants.speed_at_12_volts
        )  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(
            0.75
        )  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(
                self._max_angular_rate * 0.1
            )  # Add a 10% deadband
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )  # Use open-loop control for drive motors
        )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point = swerve.requests.PointWheelsAt()
        self._forward_straight = (
            swerve.requests.RobotCentric()
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )

        self._logger = Telemetry(self._max_speed)

        self._driver_joystick = commands2.button.CommandXboxController(0)

        self._operator_joystick = commands2.button.CommandXboxController(1)

        self.drivetrain = TunerConstants.create_drivetrain()

        self.elevator = subsystems.elevator.Elevator()

        self.limelight = subsystems.limelight.Limelight(self.drivetrain)

        # Path follower
        self._auto_chooser = AutoBuilder.buildAutoChooser("Score Coral")

        # Configure the button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -self._driver_joystick.getLeftY() * self._max_speed
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -self._driver_joystick.getLeftX() * self._max_speed
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -self._driver_joystick.getRightX() * self._max_angular_rate
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        '''self._joystick.a().whileTrue(self.drivetrain.apply_request(lambda: self._brake))
        self._joystick.b().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX())
                )
            )
        )
'''
        self._driver_joystick.pov(0).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(0.5).with_velocity_y(0)
            )
        )
        self._driver_joystick.pov(180).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(-0.5).with_velocity_y(0)
            )
        )

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        (self._driver_joystick.back() & self._driver_joystick.y()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self._driver_joystick.back() & self._driver_joystick.x()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self._driver_joystick.start() & self._driver_joystick.y()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self._driver_joystick.start() & self._driver_joystick.x()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

        # reset the field-centric heading on left bumper press
        self._driver_joystick.leftBumper().onTrue(
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric())
        )

        self.motor_speed = 0.5
        self._operator_joystick.x().whileTrue(
            self.elevator.move_command(self.motor_speed)
        )
        self._operator_joystick.y().whileTrue(
            self.elevator.move_command(-self.motor_speed))

        commands2.button.Trigger(self.is_left_trigger_pressed).whileTrue(
            subsystems.elevator.CoralOutCommand(self.elevator, self._operator_joystick.getLeftTriggerAxis)
        )
        commands2.button.Trigger(self.is_right_trigger_pressed).whileTrue(
            subsystems.elevator.CoralOutCommand(self.elevator, lambda: -self._operator_joystick.getRightTriggerAxis())
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

    def is_left_trigger_pressed(self):
        return self._operator_joystick.getLeftTriggerAxis() > 0.1 and self._operator_joystick.getRightTriggerAxis() < 0.1
    
    def is_right_trigger_pressed(self):
        return self._operator_joystick.getRightTriggerAxis()
    
    def is_right_trigger_pressed_raw(axis_value):
        return axis_value > 0.1

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        #return self._auto_chooser.getSelected()


        positions = {}
        positions[1] = Pose2d(16.40, 1.02, Rotation2d.fromDegrees(-50))
        positions[2] = Pose2d(16.41, 7.00, Rotation2d.fromDegrees(50))
        positions[3] = Pose2d(11.48, 7.55, Rotation2d.fromDegrees(90))
        positions[6] = Pose2d(13.73, 2.87, Rotation2d.fromDegrees(120))
        positions[7] = Pose2d(14.44, 4.02, Rotation2d.fromDegrees(180))
        positions[8] = Pose2d(13.77, 5.21, Rotation2d.fromDegrees(-120))
        positions[9] = Pose2d(12.31, 5.22, Rotation2d.fromDegrees(-60))
        positions[10] = Pose2d(11.70, 4.00, Rotation2d.fromDegrees(0))
        positions[11] = Pose2d(12.41, 2.81, Rotation2d.fromDegrees(60))
        positions[12] = Pose2d(1.18, 1.08, Rotation2d.fromDegrees(-127))
        positions[13] = Pose2d(1.13, 6.94, Rotation2d.fromDegrees(128))
        positions[16] = Pose2d(6.02, 0.52, Rotation2d.fromDegrees(-90))
        positions[17] = Pose2d(3.84, 2.81, Rotation2d.fromDegrees(60))
        positions[18] = Pose2d(3.04, 4.01, Rotation2d.fromDegrees(0))
        positions[19] = Pose2d(3.78, 5.24, Rotation2d.fromDegrees(-60))
        positions[20] = Pose2d(5.18, 5.19, Rotation2d.fromDegrees(-120))
        positions[21] = Pose2d(5.85, 4.03, Rotation2d.fromDegrees(180))
        positions[22] = Pose2d(5.22, 2.83, Rotation2d.fromDegrees(120))

        targetPose = positions[1]

        # Create the constraints to use while pathfinding
        constraints = PathConstraints(
            3.0,
            3.0,
            degreesToRadians(540),
            degreesToRadians(720)
        )

        # Since AutoBuilder is configured, we can use it to build pathfinding commands
        '''pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0
        )
        secondCommand = AutoBuilder.pathfindToPose(
            positions[2],
            constraints,
            0.0,
        )

        output = pathfindingCommand.andThen(secondCommand)

        return output'''

        pathfindingCommand = commands2.SequentialCommandGroup()
        for k, v in positions.items():
            command = AutoBuilder.pathfindToPose(
                v,
                constraints,
                0.0
            )
            pathfindingCommand.addCommands(command)
        
        return pathfindingCommand
            
    
    def telemetry(self):
        self.elevator.telemetry()
