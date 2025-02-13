import commands2

class MockDriveTrain(object):
    def __init__(self):
        pass

    def apply_request(self, anything) -> commands2.Command:
        return commands2.Command()
    
    def setDefaultCommand(self, anything):
        pass

    def sys_id_dynamic(self, anything) -> commands2.Command:
        return commands2.Command()
    
    def seed_field_centric(self):
        pass

    def runOnce(self, anything) -> commands2.Command:
        return commands2.Command()
    
    def register_telemetry(self, anything):
        pass