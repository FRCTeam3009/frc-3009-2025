import phoenix6.units
import ntcore
import dataclasses
from wpiutil import wpistruct
import commands2
import wpilib

@wpistruct.make_wpistruct
@dataclasses.dataclass
class ArmSegment(object):
    angle: wpistruct.double
    length: wpistruct.double

    def __init__(self, length: phoenix6.units.inch):
        self.angle = 0
        self.length = length

    def update_angle(self, delta_a: phoenix6.units.radian):
        self.angle += delta_a

    def move_command(self, angle: phoenix6.units.radian) -> commands2.Command:
        return MoveArmSegment(self, angle)
        
class MoveArmSegment(commands2.Command):
    def __init__(self, arm: ArmSegment, angle: phoenix6.units.radian):
        self.arm = arm
        self.angle = angle
        
    def execute(self):
        self.arm.update_angle(self.angle)

class CoralArm(object):
    def __init__(self):
        # TODO add shoulder arm height
        self.bicep_arm = ArmSegment(20)
        self.fore_arm = ArmSegment(20)

        # Telemetry network tables setup
        self.network_table_instance = ntcore.NetworkTableInstance.getDefault()
        self.coral_arm_table = self.network_table_instance.getTable("coralArm")
        self.bicep_publish_positions = self.coral_arm_table.getStructTopic("bicep", ArmSegment).publish()
        self.forearm_publish_positions = self.coral_arm_table.getStructTopic("forearm", ArmSegment).publish()

    def telemetry(self):
        self.bicep_publish_positions.set(self.bicep_arm)
        self.forearm_publish_positions.set(self.fore_arm)
        self.mech()

    def mech(self):
        self.box = wpilib.Mechanism2d(3, 3)
        self.root = self.box.getRoot("TheClaw", 1.5, 1.5)
        self.bicep = self.root.appendLigament(
            "Bicep", 0.5, self.bicep_arm.angle
        )
        self.shoulder = self.bicep.appendLigament(
            "Shoulder", 0.5, self.fore_arm.angle
        )
        wpilib.SmartDashboard.putData("THECLAW", self.box)