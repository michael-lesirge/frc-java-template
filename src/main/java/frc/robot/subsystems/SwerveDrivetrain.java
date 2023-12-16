package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.IntFunction;
import java.util.stream.Stream;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDrivetrainConstants;

/**
 * Subsystem for full drive train of robot. Contains 4 {@link SwerveModule} subsystems.
 * 
 * @see <a href="https://youtu.be/X2UjzPi35gU">Swerve Drive Demo</a>
 */
public class SwerveDrivetrain extends SubsystemBase {
    // Locations of wheels relative to robot center
    private static final Translation2d locationFL = new Translation2d(
            SwerveDrivetrainConstants.MODULE_LOCATION_X, SwerveDrivetrainConstants.MODULE_LOCATION_Y);
    private static final Translation2d locationFR = new Translation2d(
            SwerveDrivetrainConstants.MODULE_LOCATION_X, -SwerveDrivetrainConstants.MODULE_LOCATION_Y);
    private static final Translation2d locationBL = new Translation2d(
            -SwerveDrivetrainConstants.MODULE_LOCATION_X, SwerveDrivetrainConstants.MODULE_LOCATION_Y);
    private static final Translation2d locationBR = new Translation2d(
            -SwerveDrivetrainConstants.MODULE_LOCATION_X, -SwerveDrivetrainConstants.MODULE_LOCATION_Y);

    /**
     * The SwerveDriveKinematics class is a useful tool that converts between a ChassisSpeeds object
     * and several SwerveModuleState objects, which contains velocities and angles for each swerve module of a swerve drive robot.
     * @see https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
     */
    private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);

    /**
     * The SwerveDriveOdometry class can be used to track the position of a swerve drive robot on the field *
     * @see https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-odometry.html
     */
    private final SwerveDriveOdometry odometry;

    /** 
     * The ChassisSpeeds object represents the speeds of a robot chassis.
     * @see https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/intro-and-chassis-speeds.html#the-chassis-speeds-class
     */
    private ChassisSpeeds desiredSpeeds;

    /** 
     * True if the robot is using a field relative coordinate system, false if using a robot relive coordinate system.
     * If field relative, forward will be directly away from driver, no matter the rotation of the robot.
     * If robot relative, forward will be whatever direction the robot is facing in.
     * @see https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
     */
    private boolean isFieldRelative = false;

    // Our 4 swerve Modules
    private final SwerveModule moduleFL;
    private final SwerveModule moduleFR;
    private final SwerveModule moduleBL;
    private final SwerveModule moduleBR;

    /** All swerve modules in Java Stream Object */
    private final Stream<SwerveModule> allModules;

    /**
     * The Gyroscope on the robot. It gives data on Pitch, Yaw, and Roll of robot, as well as many other things
     * @see https://www.kauailabs.com/public_files/navx-mxp/apidocs/java/com/kauailabs/navx/frc/AHRS.html
     */
    private final AHRS gyro;

    /**
     * Pose of robot. The pose is the current the X, Y and Rotation position of the robot, relative to the last reset.
     * @see https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/pose.html#pose
     */
    private Pose2d pose;

    /**
     * Constructor the drivetrain subsystem.
     * 
     * @param gyro Gyroscope on robot, should be physically located near center of robot
     * @param swerveModuleFL Front left swerve module
     * @param swerveModuleFR Front right swerve module
     * @param swerveModuleBL Back left swerve module
     * @param swerveModuleBR Back right swerve module
     */
    public SwerveDrivetrain(AHRS gyro, SwerveModule swerveModuleFL, SwerveModule swerveModuleFR, SwerveModule swerveModuleBL, SwerveModule swerveModuleBR) {

        // save parameters
        this.gyro = gyro;

        moduleFL = swerveModuleFL;
        moduleFR = swerveModuleFR;
        moduleBL = swerveModuleBL;
        moduleBR = swerveModuleBR;
        
        allModules = Stream.of(moduleFL, moduleFR, moduleBL, moduleBR);

        // create starting state for odometry
        odometry = new SwerveDriveOdometry(
            kinematics,
            gyro.getRotation2d(),
            modules(SwerveModule::getPosition, SwerveModulePosition[]::new)
        );

        pose = new Pose2d();
    }

    /**
     * This is the periodic function of the swerve drivetrain.
     * This method is called periodically by the CommandScheduler, about every 20ms.
     */
    @Override
    public void periodic() {
        // update pose
        pose = odometry.update(
            gyro.getRotation2d(),
            modules(SwerveModule::getPosition, SwerveModulePosition[]::new));

        // display values to shuffleboard
        SmartDashboard.putBoolean("Field Relative?", isFieldRelative);

        SmartDashboard.putNumber("SpeedsX", desiredSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("SpeedsY", desiredSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("SpeedsR", desiredSpeeds.omegaRadiansPerSecond);
    }

    /** Stop all swerve modules */
    public void stop() {
        modules(SwerveModule::stop);
    }

    /**
     * Get speeds of robot.
     * <li>vx: The velocity of the robot in the x (forward) direction in meter per second.</li>
     * <li>vy: The velocity of the robot in the y (sideways) direction in meter per second. (Positive values mean the robot is moving to the left).</li>
     * <li>omega: The angular velocity of the robot in radians per second.</li>
     * 
     * @return Speeds of drivetrain (from swerve modules)
     */
    public ChassisSpeeds getState() {
        // get all module states and convert them into chassis speeds
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(modules(SwerveModule::getState, SwerveModuleState[]::new));

        if (isFieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyro.getRotation2d());
        }

        return speeds;
    }

    /**
     * Set speeds of robot.
     * <li>vx: The velocity of the robot in the x (forward) direction in meter per second.</li>
     * <li>vy: The velocity of the robot in the y (sideways) direction in meter per second. (Positive values mean the robot is moving to the left).</li>
     * <li>omega: The angular velocity of the robot in radians per second.</li>
     * 
     * @param speeds Desired speeds of drivetrain (using swerve modules)
     */
    public void setDesiredState(ChassisSpeeds speeds) {

        if (isFieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyro.getRotation2d());
        }

        this.desiredSpeeds = speeds;

        SwerveModule[] modules = modules(null, SwerveModule[]::new);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(states[i]);
        }
    }

    /** Get the desired speeds of robot */
    public ChassisSpeeds getDesiredState() {
        return desiredSpeeds;
    }

    /**
     * Get the pose of robot
     * 
     * @return The current positions of the robot, contains translational and rotational elements.
     */
    public Pose2d getPosition() {
        return pose;
    }

    /**
     * Resets the robot's pose on the field in software (reset gyro and swerve modules)
     */
    public void resetPosition() {
        odometry.resetPosition(
            gyro.getRotation2d(),
            modules(SwerveModule::getPosition, SwerveModulePosition[]::new),
            pose
        );
    }

    /**
     * Zero the yaw of the gyro. Can only change zero yaw when sensor is done calibrating
     * 
     * @return Was gyro able to zero yaw.
     */
    public boolean zeroYaw() {
        if (gyro.isCalibrating())
            return false;
        gyro.zeroYaw();
        return true;
    }

    /**
     * Set the robot's movement to either field or robot relative
     * 
     * @param fieldRelative If true, movement will be field relative
     */
    public void setFieldRelative(boolean fieldRelative) {
        this.isFieldRelative = fieldRelative;
    }

    /**
     * Toggle the robot movement between relative to the field forward and relative
     * to the robot forward
     */
    public void toggleFieldRelative() {
        isFieldRelative = !isFieldRelative;
    }

    /**
     * Check whether robot movement is relative to field or is relative to robot
     * 
     * @return True if movement is field relative
     */
    public boolean isFieldRelative() {
        return isFieldRelative;
    }

    /**
     * Get yaw (rotation) of robot
     * 
     * @return the current yaw value from -180 to 180 degrees. 0 whatever direction
     *         the robot starts in.
     */
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    /**
     * Function to easily run a function on each swerve module
     * 
     * @param func Function to run on each swerve module, takes one argument and returns nothing, operates via side effects.
     */
    private void modules(Consumer<SwerveModule> func) {
        allModules.forEach(func);
    }

    /**
     * Function to easily run a function on each swerve module and collect results to array.
     * 
     * @param <T> Type that is returned by function and should be collected
     * @param func Function that gets some data off each swerve module
     * @param arrayInitializer constructor function for array to collect results in. Use T[]::new
     * @return Array of results from func.
     */
    private <T> T[] modules(Function<? super SwerveModule, ? super T> func, IntFunction<T[]> arrayInitializer) {
        if (func == null) return allModules.toArray(arrayInitializer);
        return allModules.map(func).toArray(arrayInitializer);
    }
}
