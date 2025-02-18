from ntcore import NetworkTableInstance
import commands2.cmd
import phoenix6.swerve
import phoenix6.utils
import wpimath.geometry
import wpimath.units
import subsystems.command_swerve_drivetrain
import wpimath
import time
import wpilib

class Limelight(object):
    def __init__(self, name: str, drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain):
        default_value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # tx, ty, tz, rx, ry, rz, latency, tag count, tag span, average distance, average area


        self.nt_instance = NetworkTableInstance.getDefault()
        self.table = self.nt_instance.getTable(name)
        self.botposetopic = self.table.getDoubleArrayTopic("botpose")
        self.botposesub = self.botposetopic.subscribe(default_value)
        self.drive_train = drive_train


        self.botpose_targetspacetopic = self.table.getDoubleArrayTopic("botpose_targetspace")
        self.botpose_targetspacesub = self.botpose_targetspacetopic.subscribe([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # tx, ty, tz, rx, ry, rz
        self.line_up_timer = wpilib.Timer()        

        
        self.drive_robot_relative = (
            phoenix6.swerve.requests.RobotCentric()
            .with_drive_request_type(
                phoenix6.swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )

    def update_command(self) -> commands2.Command:
        return commands2.cmd.run(self.update).ignoringDisable(True)

    def update(self):
        self.current_bot_pose_value = self.botposesub.get()
        self.current_bot_pose_target = self.botpose_targetspacesub.get()

    def odometry_command(self) -> commands2.Command:
        return commands2.cmd.run(self.odometry_update).ignoringDisable(True)
    
    def odometry_update(self):
        botpose = self.botposesub.get()
        if self.list_check(botpose):
            pose2d = wpimath.geometry.Pose2d(botpose[0], botpose[1], botpose[5])
            latency = botpose[6]
            latency = latency / 1000.0
            seconds = time.time() - latency
            self.drive_train.add_vision_measurement(pose2d, seconds)

    def list_check(self, list):
        if len(list) < 7:
            return False

        for value in list:
            if value != 0.0:
                return True
            
        return False

    
    def lined_up(self):
        x_value = wpimath.units.metersToInches(self.current_bot_pose_target[0])
        y_value = wpimath.units.metersToInches(self.current_bot_pose_target[1])
        rotation = self.current_bot_pose_target[5]
        if phoenix6.utils.is_simulation():
            return True
        if abs(x_value - 12) <= 1:
            # TODO make this positive or negative depending on the value
            if  abs(y_value - 6.47) <= 1:
                if abs(rotation) <= 2:
                    self.line_up_timer.start()
                    if self.line_up_timer.hasElapsed(1):
                        self.line_up_timer.stop()
                        self.line_up_timer.reset()
                        return True
        self.line_up_timer.stop()
        self.line_up_timer.reset()
        return False
    
    
    def position_difference(self):
        x_value = wpimath.units.metersToInches(self.current_bot_pose_target[0])
        y_value = wpimath.units.metersToInches(self.current_bot_pose_target[1])
        rotation = self.current_bot_pose_target[5]
        return [x_value - 12, y_value - 6.47, rotation]
            
        

class line_up_coral(commands2.Command):

    def __init__(self, drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, limelight: Limelight):
        self.drive_train = drive_train
        self.limelight = limelight

    def execute(self):
        lock_on = self.limelight.position_difference()
        drive_request = lambda: self.limelight.drive_robot_relative.with_velocity_x(lock_on[0]).with_velocity_y(lock_on[1]).with_rotational_rate(lock_on[2])
        self.drive_train.apply_request(drive_request).execute()

    
    def isFinished(self):
        return self.limelight.lined_up()
    
    def end(self, interrupted):
        drive_request = lambda: self.limelight.drive_robot_relative.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(0)
        self.drive_train.apply_request(drive_request).execute()


class drive_forward_to_coral(commands2.Command):

    def __init__(self, drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, limelight: Limelight):
        self.drive_train = drive_train
        self.limelight = limelight

    def initialize(self):
        self.start_position = self.drive_train.get_state().pose
        self.end_position = self.start_position.transformBy(wpimath.geometry.Transform2d.fromFeet(11.0/12.0, 0, 0))

    def execute(self):
        drive_request = lambda: self.limelight.drive_robot_relative.with_velocity_x(0.5).with_velocity_y(0).with_rotational_rate(0)
        self.drive_train.apply_request(drive_request).execute()
        print(self.end_position)
        print(self.drive_train.get_state().pose)

    
    def isFinished(self):
        t = self.end_position - self.drive_train.get_state().pose
        distance_squared = wpimath.units.metersToInches(t.X() * t.X() + t.Y() * t.Y())
        return distance_squared < 1
    
    def end(self, interrupted):
        drive_request = lambda: self.limelight.drive_robot_relative.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(0)
        self.drive_train.apply_request(drive_request).execute()