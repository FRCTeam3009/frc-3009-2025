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
import subsystems.climber
import subsystems.controller
import subsystems.mock_drivetrain
import subsystems.solenoids
import subsystems.elevator
import telemetry
import wpimath.geometry

from phoenix6 import swerve
from wpimath.units import rotationsToRadians
import subsystems.limelight
import automodes


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
            .with_deadband(self._max_speed * 0.001)
            .with_rotational_deadband(
                self._max_angular_rate * 0.001
            )  # Add a 0.1% deadband
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

        self._logger = telemetry.Telemetry(self._max_speed)

        self._driver_joystick = commands2.button.CommandXboxController(0)

        self._operator_joystick = subsystems.controller.Controller(1)

        self.drivetrain = None
        try:
            self.drivetrain = TunerConstants.create_drivetrain()
        except Exception as e:
            print("FATAL ERROR CREATING DRIVETRAIN FROM TUNERCONSTANTS: " + str(e))
            self.drivetrain = subsystems.mock_drivetrain.MockDriveTrain()

        self.drivetrain.reset_pose(wpimath.geometry.Pose2d(10, 2, 0))

        self.elevator = subsystems.elevator.Elevator()

        self.front_limelight = subsystems.limelight.Limelight("front-limelight", self.drivetrain)
        self.back_limelight = subsystems.limelight.Limelight("back-limelight", self.drivetrain)

        self.climber = subsystems.climber.Climber()

        self.solenoids = subsystems.solenoids.Solenoids()

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
        self._driver_joystick.pov(90).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(0.0).with_velocity_y(-0.5)
            )
        )
        self._driver_joystick.pov(270).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight.with_velocity_x(0.0).with_velocity_y(0.5)
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

        commands2.button.Trigger(self._operator_joystick.is_left_stick_moved).whileTrue(
            self.elevator.move_command(self._operator_joystick.get_left_stick_y)
        )

        commands2.button.Trigger(self._operator_joystick.is_left_trigger_pressed).whileTrue(
            subsystems.elevator.CoralOutCommand(self.elevator, lambda: -1*self._operator_joystick.joystick.getLeftTriggerAxis())
        )

        commands2.button.Trigger(self._operator_joystick.is_right_trigger_pressed).whileTrue(
            subsystems.elevator.CoralOutCommand(self.elevator, lambda: self._operator_joystick.joystick.getRightTriggerAxis())
        )
        commands2.button.Trigger(self._operator_joystick.is_right_stick_moved).whileTrue(
            subsystems.elevator.coralWristCommand(self.elevator, self._operator_joystick.get_right_stick_y)
        )
        self._operator_joystick.joystick.povUp().whileTrue(
            subsystems.climber.MoveClimberCommand(self.climber, TunerConstants.climber_speed_constant)
        )
        self._operator_joystick.joystick.povDown().whileTrue(
            subsystems.climber.MoveClimberCommand(self.climber, -TunerConstants.climber_speed_constant)
        )
        self._operator_joystick.joystick.x().onTrue(
            #toggle command
            subsystems.solenoids.SolenoidsMoveCommand(self.solenoids)
            #timed command
            #subsystems.solenoids.SolenoidsMoveCommandTimed(self.solenoids)
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )
        self._driver_joystick.a().whileTrue(
            subsystems.limelight.LineUpAprilTagCommand(self.drivetrain, self.front_limelight, False)
        )
        self._driver_joystick.b().whileTrue(
            subsystems.limelight.drive_forward_command(self.drivetrain, self.front_limelight)
        )

    
    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """

        autoMode = automodes.get_auto_command(self.drivetrain, self.front_limelight, self.back_limelight, self.elevator)
        return autoMode
            
    
    def telemetry(self):
        self.elevator.telemetry()
