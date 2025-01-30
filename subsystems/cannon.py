import rev
import commands2
import RobotConfig

class CannonSubsystem(commands2.Subsystem):
    
    def __init__(self):
        self.TopMotor = rev.CANSparkMax(RobotConfig.coralCannon.TopMotorID, rev.CANSparkMax.MotorType.kBrushed)
        self.BottomMotor = rev.CANSparkMax(RobotConfig.coralCannon.BottomMotorID, rev.CANSparkMax.MotorType.kBrushed)
        self.BottomMotor.setInverted(True)
        self.pivotMotor = rev.CANSparkMax(RobotConfig.coralCannon.pivotMotorID, rev.CANSparkMax.MotorType.kBrushless)

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

