import phoenix6
from wpimath import kinematics, geometry
import wpilib
import math
from phoenix6 import hardware, controls, signals, configs
import RobotConfig as rc

class SwerveModule:

# this class defines our wheels, the swerve modules and some ways we can control them.
    def __init__(self, driveID: int, turnID: int, coderID: int, coderOffset) -> None:
        
        #create a zero position to start with
        self.desiredState = kinematics.SwerveModuleState(0, geometry.Rotation2d())
        
        #defining motor and cancoder identities on the can chain.
        self.driveMotor = hardware.TalonFX(driveID)
        self.turnMotor = hardware.TalonFX(turnID)
        self.canCoder = hardware.CANcoder(coderID)
        self.encoderOffset = coderOffset

        #defining control modes for the motors
        self.velocity = controls.VelocityVoltage(0).with_slot(0)
        self.position = controls.PositionVoltage(0).with_slot(1)
        self.brake = controls.NeutralOut() #wont actually break the motors, confusing i know

        #defining and applying configs for the cancoder
        canCoderConfigs = phoenix6.configs.CANcoderConfiguration()

        canCoderConfigs.magnet_sensor.absolute_sensor_discontinuity_point = 1
        canCoderConfigs.magnet_sensor.magnet_offset = self.encoderOffset
        
        self.canCoder.configurator.apply(canCoderConfigs)

        #defining and applying configs for motors
        MotorConfigs = phoenix6.configs.TalonFXConfiguration()

        MotorConfigs.slot0.k_p = 0.1
        MotorConfigs.slot0.k_i = 0.0
        MotorConfigs.slot0.k_d = 0.0
        MotorConfigs.slot0.k_s = 0.11
        MotorConfigs.slot0.k_v = 0.12
        MotorConfigs.voltage.peak_forward_voltage = 8
        MotorConfigs.voltage.peak_reverse_voltage = -8
        MotorConfigs.current_limits.supply_current_limit = 38 #find a real number for this
        MotorConfigs.motor_output.neutral_mode = signals.NeutralModeValue.COAST
        MotorConfigs.feedback.rotor_to_sensor_ratio = rc.SwerveModules.drivingGearRatio #check swerve stuff to get a real number

        self.driveMotor.configurator.apply(MotorConfigs)

        turnMotorConfigs = phoenix6.configs.TalonFXConfiguration()

        turnMotorConfigs.slot1.k_p = 1.0
        turnMotorConfigs.slot1.k_i = 0.0
        turnMotorConfigs.slot1.k_d = 0.0
        turnMotorConfigs.voltage.peak_forward_voltage = 13
        turnMotorConfigs.voltage.peak_reverse_voltage = -13
        turnMotorConfigs.feedback.feedback_remote_sensor_id = coderID
        turnMotorConfigs.feedback.feedback_sensor_source = signals.FeedbackSensorSourceValue.REMOTE_CANCODER
        turnMotorConfigs.motor_output.neutral_mode = signals.NeutralModeValue.COAST
        #turnMotorConfigs.feedback.sensor_to_mechanism_ratio = rc.SwerveModules.turningGearRatio #check swerve drive specs to get a real number
        turnMotorConfigs.feedback.rotor_to_sensor_ratio = rc.SwerveModules.turningGearRatio
        turnMotorConfigs.closed_loop_general.continuous_wrap = True

        #turnMotorConfigs.current_limits.supply_current_limit = 38 #find a real number for this

        self.turnMotor.configurator.apply(turnMotorConfigs)

        #starting in a zero position
        self.turnMotor.set_control(self.position.with_position(0))
        self.desiredState.angle = geometry.Rotation2d(self.turnMotor.get_position().value/ math.tau) 
    def getWheelAngleRadians(self):
        #a function i should get rid of. converts from rotations to degrees and then to radians.
        value = self.turnMotor.get_position().value / 360
        return math.radians(value)  
    
    def getTurnWheelState(self)-> geometry.Rotation2d:
        #what is the angle of the wheel?
        return geometry.Rotation2d(self.turnMotor.get_position().value / math.tau)
    
    def getDriveState(self):
        #how fast are we going?
        #not sure about if this will return as the correct data type
        return self.driveMotor.get_velocity().value * rc.driveConstants.wheelDiameter * math.pi

    def getState(self)-> kinematics.SwerveModuleState:
        #everyting one could want to know about what the wheel is doing
        return kinematics.SwerveModuleState(self.driveMotor.get_velocity().value * rc.driveConstants.wheelDiameter * math.pi, self.getTurnWheelState())
    
    def getPosition(self)-> kinematics.SwerveModulePosition:
        #where are we relative to where we started, haha nope really just the same as above
        return kinematics.SwerveModulePosition(self.driveMotor.get_position().value * rc.driveConstants.wheelDiameter * math.pi, self.getTurnWheelState())
    
    def setState(self, desiredState: kinematics.SwerveModuleState)-> None:
        #getting the wheel to where we want it to be
        optimizedDesiredState = desiredState
        optimizedDesiredState.optimize(geometry.Rotation2d(self.turnMotor.get_position().value))
        #driveMotorVelocity = optimizedDesiredState. / (rc.driveConstants.wheelDiameter * math.pi)
        #turnMotorPosition = optimizedDesiredState.angle / math.tau
        driveMotorVelocity = optimizedDesiredState.speed * math.pi
        turnMotorPosition = optimizedDesiredState.angle.radians()/ math.tau
        #print("desired positition " + str(turnMotorPosition))
        #self.turnMotor.set_control(self.velocity.with_velocity(turnMotorPosition))
        self.driveMotor.set_control(self.velocity.with_velocity(driveMotorVelocity))
        self.turnMotor.set_control(self.position.with_position(turnMotorPosition))
        #self.turnMotor.set_position(turnMotorPosition)
        self.desiredState = desiredState

    def setNuetral(self)-> None:
        #stop spinning the motors (coast them)
        self.driveMotor.set_control(self.brake)
        self.turnMotor.set_control(self.brake)
        

    def stop(self)-> None:
        #THE ROBOT MUST STOP NOW (break the motors)
        self.driveMotor.set_control(self.velocity(0))
        self.turnMotor.set_control(self.velocity(0))

    
