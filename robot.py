#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import wpilib
import commands2
import typing
import ntcore

from robotcontainer import RobotContainer

class MyRobot(commands2.TimedCommandRobot):
    """
    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """

        self.autonomousCommand: typing.Optional[commands2.Command] = None

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self.container = RobotContainer()

        self.consoleTimer = wpilib.Timer()
        self.consoleTimer.start()

        self.ntcore_instance = ntcore.NetworkTableInstance.getDefault()
        self.container_table = self.ntcore_instance.getTable("robot.py")

        self.robot_periodic_timer = wpilib.Timer()
        self.robot_periodic_topic = self.container_table.getFloatTopic("robot_periodic")
        self.robot_periodic_publish = self.robot_periodic_topic.publish()
        self.robot_periodic_publish.set(0.0)

        self.disabled_periodic_timer = wpilib.Timer()
        self.disabled_periodic_topic = self.container_table.getFloatTopic("disabled_periodic")
        self.disabled_periodic_publish = self.disabled_periodic_topic.publish()
        self.disabled_periodic_publish.set(0.0)

        self.teleop_periodic_timer = wpilib.Timer()
        self.teleop_periodic_topic = self.container_table.getFloatTopic("teleop_periodic")
        self.teleop_periodic_publish = self.teleop_periodic_topic.publish()
        self.teleop_periodic_publish.set(0.0)

        self.auto_periodic_timer = wpilib.Timer()
        self.auto_periodic_topic = self.container_table.getFloatTopic("auto_periodic")
        self.auto_periodic_publish = self.auto_periodic_topic.publish()
        self.auto_periodic_publish.set(0.0)

        self.robot_periodic_timer.reset()
        self.robot_periodic_timer.start()

    def robotPeriodic(self) -> None:
        """This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
        that you want ran during disabled, autonomous, teleoperated and test.

        This runs after the mode specific periodic functions, but before LiveWindow and
        SmartDashboard integrated updating."""

        # Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        # commands, running already-scheduled commands, removing finished or interrupted commands,
        # and running subsystem periodic() methods.  This must be called from the robot's periodic
        # block in order for anything in the Command-based framework to work.
        commands2.CommandScheduler.getInstance().run()
        
        if self.consoleTimer.hasElapsed(1):
            self.consoleTimer.reset()

        self.container.telemetry()

        self.container.elevator.sim_update()

        self.container.auto_dashboard.update()

        self.robot_periodic_publish.set(self.robot_periodic_timer.get())
        self.robot_periodic_timer.reset()

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""
        self.disabled_periodic_timer.reset()
        self.disabled_periodic_timer.start()

    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""
        self.disabled_periodic_publish.set(self.disabled_periodic_timer.get())
        self.disabled_periodic_timer.reset()

    def autonomousInit(self) -> None:
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.container.front_limelight.update_command().schedule()
        self.autonomousCommand = self.container.getAutonomousCommand()

        if self.autonomousCommand:
            self.autonomousCommand.schedule()

        self.auto_periodic_timer.reset()
        self.auto_periodic_timer.start()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""
        self.auto_periodic_publish.set(self.auto_periodic_timer.get())
        self.auto_periodic_timer.reset()

    def teleopInit(self) -> None:
        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

        self.teleop_periodic_timer.reset()
        self.teleop_periodic_timer.start()

    def teleopPeriodic(self) -> None:
        """This function is called periodically during operator control"""
        self.teleop_periodic_publish.set(self.teleop_periodic_timer.get())
        self.teleop_periodic_timer.reset()

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()
