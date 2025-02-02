import rev
import commands2
import RobotConfig

class CannonSubsystem(commands2.Subsystem):
    
    def __init__(self):
        self.intakeMotor = rev.SparkMax(RobotConfig.coralCannon.intakeMotorID, rev.SparkMax.MotorType.kBrushed)
        self.pivotMotor = rev.SparkMax(RobotConfig.coralCannon.pivotMotorID, rev.SparkMax.MotorType.kBrushless)
        

    def place(self):
        self.intakeMotor.set(1)
        
    def intake(self):
        self.intakeMotor.set(-1)

    def stop(self):
        self.intakeMotor.set(0)


    def spinup(self):
        self.pivotMotor.set(1)

    def spindown(self):
        self.pivotMotor.set(-1)

    def angleStop(self):
        self.pivotMotor.set(0)
        
