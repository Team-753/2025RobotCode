import rev
import commands2
import RobotConfig

class CannonSubsystem(commands2.Subsystem):
    
    def __init__(self):
        self.topMotor = rev.SparkMax(RobotConfig.coralCannon.TopMotorID, rev.SparkMax.MotorType.kBrushed)
        self.bottomMotor = rev.SparkMax(RobotConfig.coralCannon.BottomMotorID, rev.SparkMax.MotorType.kBrushed)
        config = rev.SparkMaxConfig()
        config.follow(RobotConfig.coralCannon.TopMotorID, True)
        self.bottomMotor.configure(config, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)
        self.pivotMotor = rev.SparkMax(RobotConfig.coralCannon.pivotMotorID, rev.SparkMax.MotorType.kBrushless)
        

    def place(self):
        print("cannon is placing")
        self.topMotor.set(0.5)
        
    def intake(self):
        self.topMotor.set(-0.5)

    def stop(self):
        self.topMotor.set(0)
    
    def idle(self):
        self.topMotor.IdleMode(0)


    def spinup(self):
        self.pivotMotor.set(0.05)

    def spindown(self):
        self.pivotMotor.set(-0.05)

    def angleStop(self):
        self.pivotMotor.set(0)
        
