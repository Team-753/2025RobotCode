import phoenix6
from wpimath import kinematics, geometry
import wpilib
import math
from phoenix6 import hardware, controls, signals
import RobotConfig as rc

class SwerveModule:

# this class defines our wheels, the swerve modules and some ways we can control them.
    def __init__(self, driveID: int, turnID: int, coderID: int) -> None:
        
        #create a zero position to start with
        self.desiredState = kinematics.SwerveModuleState(0, geometry.Rotation2d())
        
        #defining motor and cancoder identities on the can chain.
        self.driveMotor = hardware.TalonFX(driveID)
        self.turnMotor = hardware.TalonFX(turnID)
        self.canCoder = hardware.CANcoder(coderID)

        #defining control modes for the motors
        self.velocity = controls.VelocityVoltage(0).with_slot(0)
        self.position = controls.PositionVoltage(0).with_slot(0)
        self.brake = controls.NeutralOut() #wont actually break the motors, confusing i know

        #defining and applying configs for the cancoder
        canCoderConfigs = phoenix6.configs.CANcoderConfiguration()

        canCoderConfigs.magnet_sensor.absolute_sensor_discontinuity_point = 1
        
        self.canCoder.configurator.apply(canCoderConfigs)

        #defining and applying configs for motors
        driveMotorConfigs = phoenix6.configs.TalonFXConfiguration()

        driveMotorConfigs.slot0.k_p = 0.1
        driveMotorConfigs.slot0.k_i = 0.0001
        driveMotorConfigs.slot0.k_d = 0.0001
        driveMotorConfigs.slot0.k_s = 0.1
        driveMotorConfigs.slot0.k_v = 0.12
        driveMotorConfigs.voltage.peak_forward_voltage = 8
        driveMotorConfigs.voltage.peak_reverse_voltage = -8
        driveMotorConfigs.current_limits.supply_current_limit = 38 #find a real number for this
        driveMotorConfigs.motor_output.neutral_mode = signals.NeutralModeValue.COAST
        driveMotorConfigs.feedback.sensor_to_mechanism_ratio = rc.SwerveModules.drivingGearRatio #check swerve stuff to get a real number

        self.driveMotor.configurator.apply(driveMotorConfigs)

        turnMotorConfigs = phoenix6.configs.TalonFXConfiguration()

        turnMotorConfigs.slot0.k_p = 0.0005
        turnMotorConfigs.slot0.k_i = 0.0005
        turnMotorConfigs.slot0.k_d = 0.0
        turnMotorConfigs.voltage.peak_forward_voltage = 8
        turnMotorConfigs.voltage.peak_reverse_voltage = -8
        turnMotorConfigs.feedback.feedback_sensor_source = signals.FeedbackSensorSourceValue.REMOTE_CANCODER
        turnMotorConfigs.feedback.feedback_remote_sensor_id = coderID
        turnMotorConfigs.motor_output.neutral_mode = signals.NeutralModeValue.COAST
        turnMotorConfigs.feedback.sensor_to_mechanism_ratio = rc.SwerveModules.turningGearRatio #check swerve drive specs to get a real number

        turnMotorConfigs.current_limits.supply_current_limit = 38 #find a real number for this

        self.turnMotor.configurator.apply(turnMotorConfigs)

        #starting in a zero position
        self.turnMotor.set_position(0)
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
        #where are we relative to where we started
        return kinematics.SwerveModulePosition(self.driveMotor.get_position().value * rc.driveConstants.wheelDiameter * math.pi, self.getTurnWheelState())
    
    def setState(self, desiredState: kinematics.SwerveModuleState)-> None:
        #getting the wheel to where we want it to be
        #optimizedDesiredState = kinematics.SwerveModuleState.optimize(desiredState, geometry.Rotation2d(self.turnMotor.get_position().value))
        #driveMotorVelocity = optimizedDesiredState.speed / (rc.driveConstants.wheelDiameter * math.pi)
        #turnMotorPosition = optimizedDesiredState.angle / math.tau
        driveMotorVelocity = desiredState.speed * rc.SwerveModules.drivingGearRatio/ (rc.driveConstants.wheelDiameter * math.pi)
        turnMotorPosition = desiredState.angle.radians() * rc.SwerveModules.turningGearRatio/ math.tau
        self.driveMotor.set_control(self.velocity.with_velocity(driveMotorVelocity))
        self.turnMotor.set_control(self.position.with_position(turnMotorPosition))
        self.desiredState = desiredState

    def setNuetral(self)-> None:
        #stop spinning the motors (coast them)
        self.turnMotor.set_control(self.brake)
        self.turnMotor.set_control(self.brake)

    def stop(self)-> None:
        #THE ROBOT MUST STOP NOW (break the motors)
        self.driveMotor.set_control(self.velocity(0))
        self.turnMotor.set_control(self.velocity(0))

    
