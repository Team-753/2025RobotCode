import rev
import commands2
import RobotConfig

class CannonSubsystem(commands2.Subsystem):
    
    def __init__(self):
        self.intakeMotor = rev.SparkMax(RobotConfig.coralCannon.intakeMotorID, rev.SparkMax.MotorType.kBrushed)
        self.pivotMotor = rev.SparkMax(RobotConfig.coralCannon.pivotMotorID, rev.SparkMax.MotorType.kBrushless)
        

    def place(self):
        print(self.intakeMotor)
        self.intakeMotor.set(1)
        
    def intake(self):
        print(self.intakeMotor)
        self.intakeMotor.set(-1)

    def stop(self):
        self.intakeMotor.set(0)


    def spinup(self):
        print(self.pivotMotor)
        self.pivotMotor.set(1)

    def spindown(self):
        print(self.pivotMotor)
        self.pivotMotor.set(-1)

    def stop(self):
        self.pivotMotor.set(0)
        
