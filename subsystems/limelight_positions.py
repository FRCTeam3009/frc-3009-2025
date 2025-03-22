import wpimath
import wpimath.units
import wpimath.geometry
import wpilib
import subsystems.limelight

def pose2d_from_targetpose(targetpose: list[float]) -> wpimath.geometry.Pose2d:
    if len(targetpose) < 6:
        return None
        
    # [horizontal, vertical, forward, pitch, yaw, roll]
    forward = targetpose[2] # meters
    horizontal = targetpose[0] # meters
    rotation = targetpose[4] # degrees

    return wpimath.geometry.Pose2d(forward, horizontal, wpimath.units.degreesToRadians(rotation))
    
def pose2d_from_botpose(botpose: list[float]) -> wpimath.geometry.Pose2d:
    if len(botpose) < 6:
        return None
        
    # [forward, horizontal, vertical, roll, pitch, yaw, latency, tag count, tag span, average distance, average area]
    forward = botpose[0] # meters
    horizontal = botpose[1] # meters
    rotation = botpose[5] # degrees

    return wpimath.geometry.Pose2d(forward, horizontal, wpimath.units.degreesToRadians(rotation))

class SmoothPosition(object):
    def __init__(self):
        self.reset()

    def append_pose(self, pose: wpimath.geometry.Pose2d):
        if len(self.pose_list) == 0 or pose is None:
            # return early if we didn't initialize the list
            return
        
        # Remove oldest pose and add a new one to the end.
        self.pose_list.pop(0)
        self.pose_list.append(pose)

    def reset(self):
        self.pose_list = list[wpimath.geometry.Pose2d]()

        n = 0
        while n < 10:
            self.pose_list.append(wpimath.geometry.Pose2d(0.0, 0.0, 0.0))
            n += 1

    def is_zero(self):
        avg = self.get_average_pose()
        return avg.X() == 0 and avg.Y() == 0 and avg.rotation().degrees() == 0

    def get_average_pose(self):
        x = 0.0
        y = 0.0
        r = 0.0

        p : wpimath.geometry.Pose2d
        for p in self.pose_list:
            x += p.X()
            y += p.Y()
            r += p.rotation().radians()

        n = float(len(self.pose_list))
        avgx = x / n
        avgy = y / n
        avgr = r / n

        if not wpilib.RobotBase.isReal():
            # Simulation will say that we're almost lined up with the april tag offset.
            avgx = subsystems.limelight.APRIL_TAG_OFFSET + wpimath.units.inchesToMeters(6)
            avgy = -(subsystems.limelight.APRIL_TAG_OFFSET + wpimath.units.inchesToMeters(6))

        return wpimath.geometry.Pose2d(avgx, avgy, avgr)
    
def correct_target_pose(pose : wpimath.geometry.Pose2d) -> wpimath.geometry.Pose2d:
    r = -1 * pose.rotation().degrees()
    rotation = wpimath.geometry.Rotation2d.fromDegrees(r)
    horizontal = -1 * pose.Y()
    forward = pose.X()
    return wpimath.geometry.Pose2d(forward, horizontal, rotation)