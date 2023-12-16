package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;

/**
 * Subsystem for individual swerve module on robot. Each swerve module has one drive motor and one steering motor.
 * 
 * @see <a href="https://www.swervedrivespecialties.com/products/mk4-swerve-module">Swerve Module Kit</a>
 */
public class SwerveModule extends SubsystemBase {
    // the drive motor is the motor that spins the wheel making the robot move across the ground (aka velocity)
    private final CANSparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkMaxPIDController drivePIDController;

    // the steering motor is the motor that changes the rotation of the wheel allowing the robot to drive in any direction
    // Also allows for spinning in place (aka angular)
    private final CANSparkMax steeringMotor;
    private final CANCoder steeringAbsoluteEncoder;
    private final CANCoderConfiguration steeringEncoderConfiguration;
    private final PIDController steeringPIDController;

    private SwerveModuleState desiredState;

    /**
     * Constructor for an individual Swerve Module.
     * Sets up both drive and angular motor for swerve module as well as systems to monitor and control them
     * 
     * @param velocityMotorDeviceID  Device ID for drive motor
     * @param steeringMotorDeviceId  Device ID for steering motor
     * @param angularEncoderDeviceID Device ID for the angular motor's absolute encoder
     * @param steeringEncoderZero    The zero (forward) position for the angular motor's absolute encoder
     */
    public SwerveModule(int driveMotorDeviceId, int steeringMotorDeviceId, int steeringAbsoluteEncoderDeviceId,
            double steeringEncoderZero) {
        setDefaultState();

        // --- Drive Motor ---
        driveMotor = new CANSparkMax(driveMotorDeviceId, MotorType.kBrushless);
        driveMotor.setIdleMode(IdleMode.kBrake);

        // --- Drive Encoder ---
        driveEncoder = driveMotor.getEncoder();

        driveEncoder.setVelocityConversionFactor(SwerveModuleConstants.DRIVE_MOTOR_GEAR_RATIO);

        // --- Drive PID ---
        drivePIDController = driveMotor.getPIDController();
        drivePIDController.setP(SwerveModuleConstants.DRIVE_PID_P);
        drivePIDController.setI(SwerveModuleConstants.DRIVE_PID_I);
        drivePIDController.setD(SwerveModuleConstants.DRIVE_PID_D);
        drivePIDController.setFF(SwerveModuleConstants.DRIVE_PID_FF);
        drivePIDController.setIZone(0);

        drivePIDController.setOutputRange(-SwerveModuleConstants.MAX_SPEED, SwerveModuleConstants.MAX_SPEED);

        // --- Steering Motor ---
        steeringMotor = new CANSparkMax(steeringMotorDeviceId, MotorType.kBrushless);

        // --- Steering Encoder ---
        steeringEncoderConfiguration = new CANCoderConfiguration();

        // configure steering encoder to use rotations per second as unit
        steeringEncoderConfiguration.sensorCoefficient = SwerveModuleConstants.STEERING_ENCODER_SENSOR_COEFFICIENT;
        steeringEncoderConfiguration.unitString = "rot";
        steeringEncoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;

        // set offset so zero is forward
        steeringEncoderConfiguration.magnetOffsetDegrees = steeringEncoderZero;
        steeringEncoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;

        steeringAbsoluteEncoder = new CANCoder(steeringAbsoluteEncoderDeviceId, "rio");
        steeringAbsoluteEncoder.configAllSettings(steeringEncoderConfiguration);

        // --- Steering PID ---
        steeringPIDController = new PIDController(
                SwerveModuleConstants.STEERING_PID_P,
                SwerveModuleConstants.STEERING_PID_I,
                SwerveModuleConstants.STEERING_PID_D);
        steeringPIDController.enableContinuousInput(0, 360);
    }

    /**
     * This is the periodic function of the swerve module.
     * This method is called periodically by the CommandScheduler, about every 20ms.
     */
    @Override
    public void periodic() {
        // Get real and desired angles of steering motor
        // In pid the real value is often known as the measure or process variable, while the desired value is the setpoint or reference
        final double measuredDegrees = getState().angle.getDegrees();
        final double desiredDegrees = getDesiredState().angle.getDegrees();
        
        // Use the steering motor pid controller to move the motor to face the desired angle
        final double steeringMotorSpeed = steeringPIDController.calculate(measuredDegrees, desiredDegrees);
        steeringMotor.set(steeringMotorSpeed);

        // the drive motor's pid controller is in RPM so we convert our meters per second value to that
        // Use the drive motors pid controller to reach target velocity
        final double driveVelocityRotationsPerMinute = (desiredState.speedMetersPerSecond * 60) / SwerveModuleConstants.WHEEL_CIRCUMFERENCE;
        drivePIDController.setReference(driveVelocityRotationsPerMinute, CANSparkMax.ControlType.kVelocity);
    }

    /** Stop drive and steering motor of swerve module */
    public void stop() {
        driveMotor.stopMotor();
        steeringMotor.stopMotor();
    }

    /** 
     * Set the desired state of the Swerve Module to the default/starting state.
     * This should have the module facing forward and not spinning.
     */
    public void setDefaultState() {
        setDesiredState(new SwerveModuleState());
    }

    /**
     * Get the state of the swerve module. The state is the speed of our drive motor and angle of our steering motor.
     * 
     * @return Current state of swerve module, contains speed (in m/s) and angle as {@link Rotation2d}
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getDriveSpeedMetersPerSecond(),
                Rotation2d.fromRotations(getSteeringAngleRotations()));
    }

    /**
     * Set the state of the swerve module. The state is the speed and angle of the
     * swerve module. You can use {@code Rotation2d.fromDegrees()} to create angle.
     * 
     * @param state New state of swerve module, contains speed in meters per second and angle as {@link Rotation2d}
     * @param shouldOptimize Whether to optimize the way the swerve module gets to the desired state
     */
    public void setDesiredState(SwerveModuleState state, boolean shouldOptimize) {
        if (shouldOptimize) {
            // desiredState = SwerveModuleState.optimize(state, getState().angle)

            // This is a custom optimize function, since default WPILib optimize
            // (SwerveModuleState.optimize) assumes continuous controller
            desiredState = optimize(state, getState().angle);
        } else {
            desiredState = state;
        }
    }

    /**
     * Set the state of the swerve module. Will automatically optimize. The state is
     * the speed and angle of the swerve module. You can use {@code Rotation2d.fromDegrees()} to create angle.
     * 
     * @param state New state of swerve module, contains speed in meters per second and angle.
     */
    public void setDesiredState(SwerveModuleState state) {
        setDesiredState(state, true);
    }

    /**
     * Get the state of the swerve module. The state is the speed and angle of the
     * swerve module.
     */
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * Get the position of the swerve module. The position is the distance traveled by the drive motor
     * and angle of the drive motor.
     * 
     * @return Current position of swerve module, contains speed (in meters) and angle as {@link Rotation2d}
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDriveDistanceMeters(),
                Rotation2d.fromRotations(getSteeringAngleRotations()));
    }

    /**
     * Get the velocity of the drive motor.
     * 
     * @return Rotations per minute of the drive motor
     */
    private double getDriveSpeedRotationsPerMinute() {
        return driveEncoder.getVelocity();
    }

    /**
     * Get the velocity of the drive motor.
     * 
     * @return Meters per second of the drive wheel
     */
    private double getDriveSpeedMetersPerSecond() {
        return (SwerveModuleConstants.WHEEL_CIRCUMFERENCE * getDriveSpeedRotationsPerMinute()) / 60;
    }

    /**
     * Get the position of the drive motor.
     * 
     * @return Meters traveled by the drive wheel
     */
    private double getDriveDistanceMeters() {
        return SwerveModuleConstants.WHEEL_CIRCUMFERENCE * getDriveDistanceRotations();
    }

    /**
     * Get the position of the drive motor.
     * 
     * @return Total number of rotations of the drive motor
     */
    private double getDriveDistanceRotations() {
        return driveEncoder.getPosition();
    }

    /**
     * Get the angel of the steering motor.
     * 
     * @return Current position in rotations of the steering motor, accounting for offset
     */
    private double getSteeringAngleRotations() {
        return steeringAbsoluteEncoder.getPosition();
    }

    /**
     * Optimize a swerve module state so that instead of suddenly rotating the wheel (with steering motor)
     * to go a certain direction we can instead just turn a half as much and switch
     * the speed of wheel to go in reverse.
     * 
     * @param desiredState The state you want the swerve module to be in
     * @param currentAngle The current angle of the swerve module in degrees
     * @return             An optimized version of desiredState
     */
    private static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        // find the target angle in the same 0-360 scope as the desired state
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());

        // keep the same target speed
        double targetSpeed = desiredState.speedMetersPerSecond;

        // found how much we have to move to get to target angle
        double delta = targetAngle - currentAngle.getDegrees();

        // If we have to flip around more than 90 degrees than instead just reverse our direction
        // and only turn enough so that we have the motor facing in the same direction, just the other way 
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle += delta > 0 ? -180 : 180;
        }

        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    /**
     * Move an angle into the range of the reference. Finds the relative 0 and 360
     * position for a scope reference and moves the new angle into that.
     * Example: {@code placeInAppropriate0To360Scope(90, 370) = 10.0}
     * {@code placeInAppropriate0To360Scope(720, 10) = 730.0}
     * 
     * @param scopeReference The reference to find which 0-360 scope we are in. For
     *                       example 10 is in 0-360 scope while 370 is in 360-720 scope.
     * @param newAngle       The angle we want to move into found scope. For example
     *                       if the scope was 0-360 and our angle was 370 it would become 10
     * @return return the newAngle in the same scope as scopeReference
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }
}
