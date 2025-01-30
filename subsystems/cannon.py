import rev
import commands2
import RobotConfig

class CannonSubsystem(commands2.Subsystem):
    
    def __init__(self) -> None:
        self.TopMotor = rev.CANSparkMax(RobotConfig.coralCannon.TopMotorID, rev.CANSparkMax.MotorType.kBrushed)
        self.BottomMotor = rev.CANSparkMax(RobotConfig.coralCannon.BottomMotorID, rev.CANSparkMax.MotorType.kBrushed)
        self.BottomMotor.setInverted(True)

        
    def place(self) -> None:
        print(self.TopMotor)
        self.TopMotor.set(1)
        self.BottomMotor.set(1)
        
    def intake(self) -> None:
        print(self.TopMotor)
        self.TopMotor.set(-1)
        self.BottomMotor.set(-1)

    def idle(self) -> None:
        self.TopMotor.set(0)
        self.BottomMotor.set(0)
