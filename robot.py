import wpilib
from wpilib import SmartDashboard
from subsystems.vision import Vision


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):

        SmartDashboard.putNumber("Test", 5())
        SmartDashboard.putBoolean("Fasle Statement", False())
        
        self.vision_system = Vision()

#######################################
    def __init__(self) -> None:
        pass

    def autonomousInit(self):
        pass

    def autonousPeriodic(self):
        pass

    def teleopInit(self):
        pass

    def teleopPeriodic(self):
         
         SmartDashboard.putBoolean("Target Locked", self.vision_system._isTargetLocked())
    
         self.vision_system.update()

    def testInit(self):
        pass

    def testPeriodic(self):
        pass

if __name__ == "__main__":
    wpilib.run(MyRobot)
