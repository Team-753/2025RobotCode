#algae code goes here
import rev
import wpilib
import commands2
import RobotConfig as rc

class AlgaeSquisher(commands2.Subsystem):
    def init(self):
        #initializes the motor
        self.algaeMotor = rev.SparkMax(rc.algaeSquisher.squisherMotorID)
        #self.algaeMotor.restoreFactoryDefaults()
        self.algaeMotor.IdleMode(1)


        #initialize the piston
        self.pnuematicsHub = wpilib.PneumaticHub(20)
        self.algaePiston = self.pnuematicsHub.makeDoubleSolenoid(rc.algaeSquisher.squisherPistonForward, rc.algaeSquisher.squisherPistonReverse)

    

    #While calling this function rev the motor in to grab the algae
    def GrabAlgae(self):
        self.algaeMotor.set(1)
    
    #rev out the algae into processor
    def ReleaseAlgae(self):
        self.algaeMotor.set(-1)
    
    #pull the algae squisher back up
    def PullPistonBack(self):
        self.algaePiston.set(wpilib.DoubleSolenoid.Value.kReverse)

     #extend piston out 
    def ExtendPiston(self):
        self.algaePiston.set(wpilib.DoubleSolenoid.Value.kForward)