import rev
import commands2
import RobotConfig

class CannonSubsystem(commands2.Subsystem):
    
    def __init__(self, auxController: commands2.button.CommandXboxController):
        self.topMotor = rev.SparkMax(RobotConfig.coralCannon.TopMotorID, rev.SparkMax.MotorType.kBrushed)
        self.bottomMotor = rev.SparkMax(RobotConfig.coralCannon.BottomMotorID, rev.SparkMax.MotorType.kBrushed)
        config = rev.SparkMaxConfig()
        config.follow(RobotConfig.coralCannon.TopMotorID, True)
        self.bottomMotor.configure(config, rev.SparkMax.ResetMode.kNoResetSafeParameters, rev.SparkMax.PersistMode.kNoPersistParameters)
        self.pivotMotor = rev.SparkMax(RobotConfig.coralCannon.pivotMotorID, rev.SparkMax.MotorType.kBrushless)
        
        self.controller = auxController

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
        self.pivotMotor.set(-0.1)

    def spindown(self):
        self.pivotMotor.set(0.1)

    def angleStop(self):
        self.pivotMotor.IdleMode(1)
    def angleIdle(self):
        self.pivotMotor.set(0)
    def GetJoystickInput(self):
        return(-self.controller.getRightY())
    
    def ManualControl(self, controllerInput):
        self.joystick = controllerInput
        if(abs(self.joystick) > .5):
            if(self.joystick > 0):
                #Lowers the elevator motors
                self.spinup()
                print("spinning up")
            elif(self.joystick < 0):
                #sets the motor to raise
                self.spindown()
                print("spinning down")
        else:
            self.angleIdle()
