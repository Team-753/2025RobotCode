import rev
import commands2
import RobotConfig

class CannonSubsystem(commands2.Subsystem):
    
    def __init__(self):
        self.TopMotor = rev.SparkMax(RobotConfig.coralCannon.TopMotorID, rev.SparkMax.MotorType.kBrushed)
        self.BottomMotor = rev.SparkMax(RobotConfig.coralCannon.BottomMotorID, rev.SparkMax.MotorType.kBrushed)
        self.BottomMotor.setInverted(True)
        self.pivotMotor = rev.SparkMax(RobotConfig.coralCannon.pivotMotorID, rev.SparkMax.MotorType.kBrushless)

    def place(self):
        print(self.TopMotor)
        self.TopMotor.set(1)
        self.BottomMotor.set(1)
        
    def intake(self):
        print(self.TopMotor)
        self.TopMotor.set(-1)
        self.BottomMotor.set(-1)

    def idle(self):
        self.TopMotor.set(0)
        self.BottomMotor.set(0)

    def pivotup(self):
        print(self.pivotMotor)
        self.pivotMotor.set(1)

    def pivotdown(self):
        print(self.pivotMotor)
        self.pivotMotor.set(-1)

