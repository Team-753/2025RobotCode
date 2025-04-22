import phoenix6
from wpimath import kinematics, geometry
import wpilib
import math
from phoenix6 import hardware, controls, signals, configs
import RobotConfig as rc


#This is where I put the swerve code from the new 2025 robot
class SwerveModule:

# this class defines our wheels, the swerve modules and some ways we can control them.
    def __init__(self, driveID: int, turnID: int, coderID: int, coderOffset: float, invertedness: bool) -> None:
        
      
        
        #defining motor and cancoder identities on the can chain.
        self.driveMotor = hardware.TalonFX(driveID)
        self.turnMotor = hardware.TalonFX(turnID)
        self.canCoder = hardware.CANcoder(coderID)
        self.encoderOffset = coderOffset
        self.inverted = invertedness

        #create a zero position to start with, basically tells the wheel that whatever position it is in is the zero pose
        self.desiredState = kinematics.SwerveModuleState(0, geometry.Rotation2d(0))


        #defining control modes for the motors
        self.velocity = controls.VelocityVoltage(0).with_slot(0) #this is used to control the drive motor during teleop when we want to control it via speed. Also currently used in auto for a-stop and for the fancy autos.
        self.position = controls.PositionVoltage(0).with_slot(1) #this is used to control the turn (azimuth) motor, also the control mode that would need to be used for the drive motor if we used pathplanner
        # we use the velocity when it makes the most sense to control the robot via the speed. the fancy auto commands control it using pids, which are speed depenedent. The position is good when we need the motor to got to as specific place, rather than a velocity. That is why we use it for the turn motor and for some autos, you can basically tell it to go somewhere specific on the field.
        self.brake = controls.NeutralOut() #wont actually break the motors, confusing i know. stops sending power to the motors, but allows the shaft to rotate freely. this differs from brake because break has the motor hold the shaft in place, which is bad because it can cause the motor to stall and make magic smoke.

        #defining and applying configs for the cancoder
        canCoderConfigs = phoenix6.configs.CANcoderConfiguration() #a class that stores settings for the CANcoders

        canCoderConfigs.magnet_sensor.absolute_sensor_discontinuity_point = 1 # sets the cancoder to count from 0 to 2pi, rather than -pi to pi, which is compatible with other libraries better
        canCoderConfigs.magnet_sensor.magnet_offset = self.encoderOffset # sets the zero position of the CAN coder to be the zero position that we want it to be, not the semi-arbitrary default. This lets us have all the wheels line up with the robot chassis by default, and lets us drive
        canCoderConfigs.magnet_sensor.sensor_direction = signals.SensorDirectionValue.CLOCKWISE_POSITIVE #can coder invert, I did this so that the wheels would align to spin the robot when the z axis on the joystick was manipulated, otherwise they made an x
        
        self.canCoder.configurator.apply(canCoderConfigs) # sending the settings to the CANcoder

        #defining and applying configs for motors
        MotorConfigs = phoenix6.configs.TalonFXConfiguration() # class containing settings for the drive motor

        MotorConfigs.slot0.k_p = 1
        MotorConfigs.slot0.k_i = 0.0
        MotorConfigs.slot0.k_d = 0.0
        MotorConfigs.slot0.k_s = 0.11
        MotorConfigs.slot0.k_v = 0.12
        #These are the pid configs for the drive motor. they are mostly default values. they are used to control the acceleration curve of the drive motor through an algorythmn that compares how fast the wheel is going with how fast we want it to go. if you need more help with this wpi and ctre have some resources

        MotorConfigs.voltage.peak_forward_voltage = 13
        MotorConfigs.voltage.peak_reverse_voltage = -13
        # These two configs limit the amount of voltage that the motors can draw, which can be an indirect way to limit speed, or it can help ensure battery longevity by reducing power draw
        MotorConfigs.current_limits.supply_current_limit = 38 # Not quite sure what this does, but it can help to reduce brownouts
        MotorConfigs.motor_output.neutral_mode = signals.NeutralModeValue.COAST # when we arent trying to do anything with the motor we set it to coast mode, which stops the motor but allows the shaft to rotate freely, which doesnt let the motor stall
        #MotorConfigs.feedback.rotor_to_sensor_ratio = rc.SwerveModules.drivingGearRatio
        if driveID == 1 or 7:
            MotorConfigs.feedback.sensor_to_mechanism_ratio = rc.SwerveModules.drivingGearRatio
        elif driveID == 4 or 10:
            MotorConfigs.feedback.sensor_to_mechanism_ratio = 5.60

        #telling the motor how many spins of the motor per desired mechanism output spin. this way we can talk to it in mechanism spins, rather than having to do that math later

        #The following configs are for position based drive... might be helpful for autos? Long story short, I never used these, they are default values but an interesting place to start if you want to use pathplanner
        MotorConfigs.slot1.k_p = 7.2
        MotorConfigs.slot1.k_i = 2.4
        MotorConfigs.slot1.k_d = 0.0001
        MotorConfigs.slot1.k_s = 0.01

        self.driveMotor.configurator.apply(MotorConfigs) #sending the config to the drive motor

        turnMotorConfigs = phoenix6.configs.TalonFXConfiguration() #class containing configs for the turn motor

        turnMotorConfigs.slot1.k_p = 7.2
        turnMotorConfigs.slot1.k_i = 2.4
        turnMotorConfigs.slot1.k_d = 0.0001
        turnMotorConfigs.slot1.k_s = 0.01
        #same as in the drive motor configs, but with slightly different values because we are comparing current position with desired position, instead of comparing velocities. Tune these values with the robot (or test bot) on the ground. If youve got the test bot, i reccomend sticking a weight on it (50lb?) to better simulate the real robot and give you a better impression of the amount of oscillation you will actually see

        turnMotorConfigs.voltage.peak_forward_voltage = 13
        turnMotorConfigs.voltage.peak_reverse_voltage = -13
        #literally the exact same as the purpose of these configs in the drive motor, reduce brownouts

        turnMotorConfigs.feedback.feedback_remote_sensor_id = coderID #tells the turn motor to use the encoder at the specified place on the CAN chain rather than its built in encoder. This lets us use the CAN coders
        turnMotorConfigs.feedback.feedback_sensor_source = signals.FeedbackSensorSourceValue.REMOTE_CANCODER #tells the motor that it will be looking at a CAN coder at that specified place on the can chain. also tells it to always use the can coder instead of the built in encoder
        turnMotorConfigs.motor_output.neutral_mode = signals.NeutralModeValue.COAST #when were not trying to do anything, dont move, but the shaft can move cause stalling the motor out is bad
        #turnMotorConfigs.motor_output.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE #trying to fix backward swerve
        #turnMotorConfigs.feedback.sensor_to_mechanism_ratio = rc.SwerveModules.turningGearRatio #check swerve drive specs to get a real number
        #turnMotorConfigs.feedback.rotor_to_sensor_ratio = rc.SwerveModules.turningGearRatio
        turnMotorConfigs.closed_loop_general.continuous_wrap = True # the motor controls something that goes around in a circle (as opposed to an elevtor), sp 0 and 2pi are the same. Step one of minimizing turning of the wheels.
        #turnMotorConfigs.feedback.feedback_rotor_offset = self.canCoder.get_absolute_position().value
        

        turnMotorConfigs.current_limits.supply_current_limit = 38 #find a real number for this

        self.turnMotor.configurator.apply(turnMotorConfigs) #send the turn motor configs to the turn motor

        #print("can coder position " + str(self.canCoder.get_absolute_position().value))
        self.turnMotor.set_position((self.canCoder.get_absolute_position().value))
        #print(" old mechanism position " + str(self.turnMotor.get_position().value))
        #print("rotor position " + str(self.turnMotor.get_rotor_position().value))
        '''if self.inverted == True:
            turnMotorConfigs.motor_output.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE'''
        
        self.turnMotor.set_control(self.position.with_position(self.turnMotor.get_position().value))
        #debugging stuff, basically setting the turn motor position to zero internally, and then moving to the zero position

        self.desiredState.angle = geometry.Rotation2d() #the current desired pose is the current location of the wheel
          
        
        #starting in a zero position
        #self.turnMotor.set_control(self.position.with_position(0))
        #self.setState(self.desiredState) 
        #print("**********")
    def getWheelAngleRadians(self):
        #a function i should get rid of. converts from rotations to degrees and then to radians. This is never actually used
        value = self.turnMotor.get_position().value / (360) 
        return math.radians(value)  
    
    def getTurnWheelState(self)-> geometry.Rotation2d:
        """Returns the current position of the turn motor in radians"""
        #what is the angle of the wheel?
        return geometry.Rotation2d(self.turnMotor.get_position().value * (math.tau))
    
    def getDriveState(self):
        """Returns the current velocity of the wheel in m/s"""
        #how fast are we going?
        #not sure about if this will return as the correct data type
        return self.driveMotor.get_velocity().value * rc.driveConstants.wheelDiameter * math.pi

    def getState(self)-> kinematics.SwerveModuleState:
        """Returns the current velocity of the wheels in m/s and the current position of the wheel in radians as a SwerveModuleState object to be used in calculations for setting the motors"""
        #everyting one could want to know about what the wheel is doing
        return kinematics.SwerveModuleState(self.driveMotor.get_velocity().value * rc.driveConstants.wheelDiameter * math.pi, self.getTurnWheelState())
    
    def getPosition(self)-> kinematics.SwerveModulePosition:
        """Returns the current position of the wheels relative to the zero location on the field in m and the current turn location of the wheels in radians. good for some auto calculations"""
        #where are we relative to where we started, haha nope really just the same as above
        return kinematics.SwerveModulePosition(self.driveMotor.get_position().value * rc.driveConstants.wheelDiameter * math.pi, self.getTurnWheelState())
    
    def setState(self, optimizedDesiredState: kinematics.SwerveModuleState)-> None:
        #getting the wheel to where we want it to be
        optimizedDesiredState.optimize(geometry.Rotation2d(self.turnMotor.get_position().value)) #This fuction, along with the continuous wrap being set to true in the config for the turn motor ensures that the wheel never turns more than 90 degrees relative to the robot chassis. This line in particular lets us invert the direction of the drive motor to make it seem like it had turned an additional 90 degrees
        #driveMotorVelocity = optimizedDesiredState. / (rc.driveConstants.wheelDiameter * math.pi)
        #turnMotorPosition = optimizedDesiredState.angle / math.tau
        driveMotorVelocity = optimizedDesiredState.speed # getting the speed we should set the motor to from the optimized desired state object and saving it to its own variable
        turnMotorPosition = optimizedDesiredState.angle.radians() / math.tau # getting the desired angle from the optimized desired state and saving it as its own variable
        '''print("module number " +  str(self.driveMotor.device_id))
        print ("desired positition " + str(turnMotorPosition))
        print ("current position " + str(self.turnMotor.get_position().value))
        print ("new CanCoder position " + str(self.canCoder.get_absolute_position().value))'''
        #self.turnMotor.set_control(self.velocity.with_velocity(turnMotorPosition))
        self.driveMotor.set_control(self.velocity.with_velocity(driveMotorVelocity)) #setting the drive motor to go to the desired velocity
        self.turnMotor.set_control(self.position.with_position(turnMotorPosition)) #setting the turn motor to the desired position
        
    def setNuetral(self)-> None:
        """Sets the motors to neutral mode to coast the robot"""
        #stop spinning the motors (coast them)
        self.driveMotor.set_control(self.brake)
        self.turnMotor.set_control(self.brake)
        

    def stop(self)-> None:
        """Sets the velocity of both motors to zero, which should brake them"""
        #THE ROBOT MUST STOP NOW (break the motors)
        self.driveMotor.set_control(self.velocity.with_velocity(0))
        self.turnMotor.set_control(self.velocity.with_velocity(0))

   



