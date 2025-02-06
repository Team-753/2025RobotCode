import wpilib
from subsystems.vision import Vision

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.vision_system = Vision()

    def teleopPeriodic(self):
        # Just call update() each loop. Vision handles everything internally
        self.vision_system.update()

    # All other methods can remain empty or add your own logic
    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass
    
    def teleopInit(self):
        pass

    def testInit(self):
        pass

    def testPeriodic(self):
        pass
    
    def robotPeriodic(self):
        pass

    def disabledPeriodic(self):
        pass

if __name__ == "__main__":
    wpilib.run(MyRobot)
