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

CORAL_POST_OFFSET = 6.47 # inches offset from center of AprilTag


class Limelight(object):
    def __init__(self, name: str, drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain):
        default_value = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # tx, ty, tz, rx, ry, rz, latency, tag count, tag span, average distance, average area


        self.nt_instance = NetworkTableInstance.getDefault()
        self.table = self.nt_instance.getTable(name)
        self.botposetopic = self.table.getDoubleArrayTopic("botpose_targetspace")
        self.botposesub = self.botposetopic.subscribe(default_value)
        self.drive_train = drive_train

        self.current_bot_pose_field = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.current_bot_pose_target = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.botpose_targetspacetopic = self.table.getDoubleArrayTopic("test_botpose_targetspace")
        self.botpose_targetspacesub = self.botpose_targetspacetopic.subscribe([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]) # tx, ty, tz, rx, ry, rz

        
        self.drive_robot_relative = (
            phoenix6.swerve.requests.RobotCentric()
            .with_drive_request_type(
                phoenix6.swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )


        self.lined_up_topic = self.table.getBooleanTopic("test lined up")
        self.lined_up_publish = self.lined_up_topic.publish()
        self.lined_up_publish.set(False)

        self.bot_pose_topic = self.table.getDoubleArrayTopic("test current bot pose")
        self.bot_pose_publish = self.bot_pose_topic.publish()
        self.bot_pose_publish.set([0.0, 0.0, 0.0])

    def update_command(self) -> commands2.Command:
        return commands2.cmd.run(self.update).repeatedly().ignoringDisable(True)

    def update(self):
        fieldspace = self.botposesub.get()
        if self.list_check(fieldspace):
            self.current_bot_pose_field = fieldspace

        targetspace = self.botpose_targetspacesub.get()
        if self.list_check(targetspace):
            self.current_bot_pose_target = targetspace

    def odometry_command(self) -> commands2.Command:
        return commands2.cmd.run(self.odometry_update).repeatedly().ignoringDisable(True)
    
    def odometry_update(self):
        botpose = self.current_bot_pose_field
        if self.list_check(botpose):
            pose2d = wpimath.geometry.Pose2d(botpose[0], botpose[1], botpose[5])
            seconds = 0
            if len(botpose) > 6:
                latency = botpose[6]
                latency = latency / 1000.0
                seconds = time.time() - latency
            self.drive_train.add_vision_measurement(pose2d, seconds)

    def list_check(self, l):
        if len(l) < 6:
            return False

        for value in l:
            if value != 0.0:
                return True
            
        return False

    
    def lined_up(self, offset):
        x_value = wpimath.units.metersToInches(self.current_bot_pose_target[0])
        y_value = wpimath.units.metersToInches(self.current_bot_pose_target[1])
        rotation = self.current_bot_pose_target[5]
        if phoenix6.utils.is_simulation():
            return True
        if abs(x_value - 12) <= 1:
            if  abs(y_value - offset) <= 1:
                if abs(rotation) <= 2:
                    return True
        return False
    
    
    def position_difference(self, offset):
        x_value = wpimath.units.metersToInches(self.current_bot_pose_target[0])
        y_value = wpimath.units.metersToInches(self.current_bot_pose_target[1])
        rotation = self.current_bot_pose_target[5]

        horizontal = offset # inches left/right (left is positive due to coordinate system)
        
        forward = -12 # inches away from wall

        return [x_value + forward, y_value + horizontal, rotation]
    
    def telemetry(self):
        self.lined_up_publish.set(self.lined_up(6.47))
        self.bot_pose_target_var = [wpimath.units.metersToInches(self.current_bot_pose_target[0]), 
                                    wpimath.units.metersToInches(self.current_bot_pose_target[1]), 
                                    wpimath.units.metersToInches(self.current_bot_pose_target[5])]
        self.bot_pose_target_var = self.botpose_targetspacesub.get()
        self.bot_pose_publish.set(self.bot_pose_target_var)
        
            
        

class LineUpAprilTagCommand(commands2.Command):

    def __init__(self,
                 drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                 limelight: Limelight,
                 offset: float):
        self.drive_train = drive_train
        self.limelight = limelight
        self.offset = offset

    def execute(self):
        lock_on = self.limelight.position_difference(self.offset)
        x = lock_on[0]
        y = lock_on[1]
        z = lock_on[2]
        forward = 0
        horizontal = 0
        rotation = 0
        if x > 0:
            forward = -0.2
        elif x < 0:
            forward = 0.2
        if y > 0:
            horizontal = -0.2
        elif y < 0:
            horizontal = 0.2
        if z < 0:
            rotation = 0.5
        elif z > 0:
            rotation = -0.5
        drive_request = lambda: self.limelight.drive_robot_relative.with_velocity_x(forward).with_velocity_y(horizontal).with_rotational_rate(rotation)
        self.drive_train.apply_request(drive_request).execute()

    
    def isFinished(self):
        return self.limelight.lined_up(self.offset)
    
    def end(self, interrupted):
        drive_request = lambda: self.limelight.drive_robot_relative.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(0)
        self.drive_train.apply_request(drive_request).execute()

def line_up_left_post_command(drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain,
                              limelight: Limelight,
                            ):
    return LineUpAprilTagCommand(drive_train, limelight, CORAL_POST_OFFSET)

def line_up_right_post_command(drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain,
                              limelight: Limelight,
                            ):
    return LineUpAprilTagCommand(drive_train, limelight, -CORAL_POST_OFFSET)

def line_up_for_coral_pickup(drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain,
                              limelight: Limelight,
                            ):
    return LineUpAprilTagCommand(drive_train, limelight, 0.0)

class DriveStraightCommand(commands2.Command):

    def __init__(self, 
                 drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain, 
                 limelight: Limelight,
                 distance: float,
                 speed: float
                 ):
        self.drive_train = drive_train
        self.limelight = limelight
        self.distance = distance
        self.speed = speed

    def initialize(self):
        self.start_position = self.drive_train.get_state().pose
        self.end_position = self.start_position.transformBy(wpimath.geometry.Transform2d.fromFeet(self.distance, 0, 0))

    def execute(self):
        drive_request = lambda: self.limelight.drive_robot_relative.with_velocity_x(self.speed).with_velocity_y(0).with_rotational_rate(0)
        self.drive_train.apply_request(drive_request).execute()

    
    def isFinished(self):
        t = self.end_position - self.drive_train.get_state().pose
        distance_squared = wpimath.units.metersToInches(t.X() * t.X() + t.Y() * t.Y())
        close = 1 # inches
        return distance_squared < close * close
    
    def end(self, interrupted):
        drive_request = lambda: self.limelight.drive_robot_relative.with_velocity_x(0).with_velocity_y(0).with_rotational_rate(0)
        self.drive_train.apply_request(drive_request).execute()

def drive_forward_command(drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain,
                          limelight: Limelight):
    distance = 11.0/12.0
    speed = 0.5
    return DriveStraightCommand(drive_train, limelight, distance, speed)

def drive_backward_command(drive_train: subsystems.command_swerve_drivetrain.CommandSwerveDrivetrain,
                           limelight: Limelight):
    distance = -11.0/12.0
    speed = -0.5
    return DriveStraightCommand(drive_train, limelight, distance, speed)