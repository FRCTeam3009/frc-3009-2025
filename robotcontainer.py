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
import subsystems.drive_robot_relative
import subsystems.shooter
import subsystems.elevator
import subsystems.wrist
import telemetry
import wpilib
import ntcore
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

        self.drivetrain = TunerConstants.create_drivetrain()

        self.drivetrain.reset_pose(wpimath.geometry.Pose2d(10, 2, 0))

        self.elevator = subsystems.elevator.Elevator()

        self.wrist = subsystems.wrist.Wrist()

        self.shooter = subsystems.shooter.Shooter()

        self.front_limelight = subsystems.limelight.Limelight("limelight-front", self.drivetrain)
        self.back_limelight = subsystems.limelight.Limelight("limelight-back", self.drivetrain)

        self.climber = subsystems.climber.Climber()

        self.auto_dashboard = automodes.AutoDashboard()

        self.periodic_timer = wpilib.Timer()
        self.periodic_timer.start()

        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.container_table = self.ntcore_instance.getTable("robotcontainer")
        self.periodic_topic = self.container_table.getFloatTopic("periodic_timer")
        self.periodic_publish = self.periodic_topic.publish()
        self.periodic_publish.set(0.0)

        self.speed_limit = subsystems.drive_robot_relative.NORMAL_SPEED

        # Configure the button bindings
        self.configureButtonBindings()

        commands2.CommandScheduler.getInstance().setDefaultCommand(self.elevator, subsystems.elevator.HoldPositionCommand(self.elevator))
        commands2.CommandScheduler.getInstance().setDefaultCommand(self.climber, subsystems.climber.MoveClimberCommand(self.climber, 0.0))
        commands2.CommandScheduler.getInstance().setDefaultCommand(self.wrist, subsystems.wrist.HoldPositionCommand(self.wrist))
        commands2.CommandScheduler.getInstance().setDefaultCommand(self.shooter, subsystems.shooter.HoldShooter(self.shooter))
        
        self.front_limelight.update_command().schedule()
        self.back_limelight.update_command().schedule()
        #self.front_limelight.odometry_command().schedule()

    def set_turbo_speed(self):
        self.speed_limit = subsystems.drive_robot_relative.TURBO_SPEED

    def set_normal_speed(self):
        self.speed_limit = subsystems.drive_robot_relative.NORMAL_SPEED

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        self._driver_joystick.rightTrigger().whileTrue(
            commands2.cmd.run(self.set_turbo_speed).finallyDo(lambda interrupted: self.set_normal_speed())
        )

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        -self._driver_joystick.getLeftY() * self._max_speed * self.speed_limit
                    )  # Drive forward with negative Y (forward)
                    .with_velocity_y(
                        -self._driver_joystick.getLeftX() * self._max_speed * self.speed_limit
                    )  # Drive left with negative X (left)
                    .with_rotational_rate(
                        -self._driver_joystick.getRightX() * self._max_angular_rate * self.speed_limit
                    )  # Drive counterclockwise with negative X (left)
                )
            )
        )

        self._driver_joystick.povUp().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight
                .with_velocity_x(subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_velocity_y(0)
                .with_rotational_rate(-self._driver_joystick.getRightX() * self._max_angular_rate / 2)
            )
        )
        self._driver_joystick.povDown().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight
                .with_velocity_x(-subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_velocity_y(0)
                .with_rotational_rate(-self._driver_joystick.getRightX() * self._max_angular_rate / 2)
            )
        )
        self._driver_joystick.povRight().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight
                .with_velocity_x(0.0)
                .with_velocity_y(-subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_rotational_rate(-self._driver_joystick.getRightX() * self._max_angular_rate / 2)
            )
        )
        self._driver_joystick.povLeft().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight
                .with_velocity_x(0.0)
                .with_velocity_y(subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_rotational_rate(-self._driver_joystick.getRightX() * self._max_angular_rate / 2)
            )
        )
        self._driver_joystick.povUpRight().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight
                .with_velocity_x(subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_velocity_y(-subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_rotational_rate(-self._driver_joystick.getRightX() * self._max_angular_rate / 2)
            )
        )
        self._driver_joystick.povDownRight().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight
                .with_velocity_x(-subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_velocity_y(-subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_rotational_rate(-self._driver_joystick.getRightX() * self._max_angular_rate / 2)
            )
        )
        self._driver_joystick.povDownLeft().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight
                .with_velocity_x(-subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_velocity_y(subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_rotational_rate(-self._driver_joystick.getRightX() * self._max_angular_rate / 2)
            )
        )
        self._driver_joystick.povUpLeft().whileTrue(
            self.drivetrain.apply_request(
                lambda: self._forward_straight
                .with_velocity_x(subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_velocity_y(subsystems.drive_robot_relative.NORMAL_SPEED)
                .with_rotational_rate(-self._driver_joystick.getRightX() * self._max_angular_rate / 2)
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
            subsystems.elevator.MoveElevatorCommand(self.elevator, lambda: self._operator_joystick.get_left_stick_y() * subsystems.elevator.SPEED)
        )

        # Coral intake and Algae shoot
        commands2.button.Trigger(self._operator_joystick.is_left_trigger_pressed).whileTrue(
            subsystems.shooter.CoralOutCommand(self.shooter, lambda: self._operator_joystick.joystick.getLeftTriggerAxis()*subsystems.shooter.SPEED)
        )

        # Coral shoot and Algae intake
        commands2.button.Trigger(self._operator_joystick.is_right_trigger_pressed).whileTrue(
            subsystems.shooter.CoralOutCommand(self.shooter, lambda: -1*self._operator_joystick.joystick.getRightTriggerAxis()*subsystems.shooter.SPEED)
        )
        commands2.button.Trigger(self._operator_joystick.is_right_stick_moved).whileTrue(
            subsystems.wrist.IncrementWrist(self.wrist, lambda: self._operator_joystick.get_right_stick_y() * self.wrist_speed(), self.elevator)
        )
        self._operator_joystick.joystick.povUp().whileTrue(
            subsystems.climber.MoveClimberCommand(self.climber, subsystems.climber.SPEED)
        )
        self._operator_joystick.joystick.povDown().whileTrue(
            subsystems.climber.MoveClimberCommand(self.climber, -1 * subsystems.climber.SPEED)
        )
        self._operator_joystick.joystick.a().whileTrue(
            subsystems.wrist.CoralWristToPosition(self.wrist, subsystems.wrist.CoralWristToPosition.pickup, self.elevator)
        )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )
        self._driver_joystick.b().whileTrue(
            subsystems.drive_robot_relative.drive_forward_command(self.drivetrain, subsystems.drive_robot_relative.FORWARD_OFFSET, self.speed_limit)
        )
        self._operator_joystick.joystick.povLeft().whileTrue(
            subsystems.wrist.CoralWristToPosition(self.wrist, subsystems.wrist.CoralWristToPosition.L1, self.elevator).alongWith(
            subsystems.elevator.MoveElevatorToPosition(self.elevator, subsystems.elevator.MoveElevatorToPosition.L1))
        )
        self._operator_joystick.joystick.y().whileTrue(
            subsystems.wrist.CoralWristToPosition(self.wrist, subsystems.wrist.CoralWristToPosition.L3, self.elevator).alongWith(
            subsystems.elevator.MoveElevatorToPosition(self.elevator, subsystems.elevator.MoveElevatorToPosition.L3))
        )
        self._operator_joystick.joystick.x().whileTrue(
            subsystems.wrist.CoralWristToPosition(self.wrist, subsystems.wrist.CoralWristToPosition.L2, self.elevator).alongWith(
            subsystems.elevator.MoveElevatorToPosition(self.elevator, subsystems.elevator.MoveElevatorToPosition.L2))
        )
        self._operator_joystick.joystick.b().whileTrue(
            #subsystems.wrist.CoralWristToPosition(self.wrist, subsystems.wrist.CoralWristToPosition.L4, self.elevator).alongWith(
            #subsystems.elevator.MoveElevatorToPosition(self.elevator, subsystems.elevator.MoveElevatorToPosition.L4))
            subsystems.wrist.CoralWristToPosition(self.wrist, subsystems.wrist.CoralWristToPosition.ground_pickup, self.elevator)
        )
        self._operator_joystick.joystick.leftBumper().onTrue(
            subsystems.wrist.MoveIntake(self.wrist.intake_servo)
        )
        self._operator_joystick.joystick.rightBumper().onTrue(
            subsystems.shooter.HalfShot(self.shooter)
        )

    
    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """

        autoMode = self.auto_dashboard.get_current_auto_builder(self.drivetrain, self.front_limelight, self.back_limelight, self.elevator, self.wrist, self.shooter)
        return autoMode

    def wrist_speed(self) -> float:
        speed = subsystems.wrist.DRIVE_SPEED
        if self._operator_joystick.joystick.back().getAsBoolean():
            speed = subsystems.wrist.TURBO_SPEED

        return speed
    
    def telemetry(self):
        self.climber.telemetry()
        self.elevator.telemetry()
        self.wrist.telemetry()
        self.periodic_publish.set(self.periodic_timer.get())
        self.periodic_timer.reset()
        self.front_limelight.telemetry()

        