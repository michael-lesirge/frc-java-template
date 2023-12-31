package frc.robot.subsystems;

import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.IntFunction;
import java.util.stream.Stream;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for full drive train of robot. Contains 4 {@link SwerveModule} subsystems.
 * 
 * @see <a href="https://youtu.be/X2UjzPi35gU">Swerve Drive Demo</a>
 */
public class SwerveDrivetrain extends SubsystemBase {
    /**
     * The SwerveDriveKinematics class is a useful tool that converts between a ChassisSpeeds object
     * and several SwerveModuleState objects, which contains velocities and angles for each swerve module of a swerve drive robot.
     * @see https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
     */
    private final SwerveDriveKinematics kinematics;

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

    // Our 4 swerve Modules
    private final SwerveModule moduleFL;
    private final SwerveModule moduleFR;
    private final SwerveModule moduleBL;
    private final SwerveModule moduleBR;

    /**
     * The Gyroscope on the robot. It gives data on Pitch, Yaw, and Roll of robot, as well as many other things
     * @see https://www.kauailabs.com/public_files/navx-mxp/apidocs/java/com/kauailabs/navx/frc/AHRS.html
     * @see https://ibb.co/dJrL259
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
        
        kinematics = new SwerveDriveKinematics(
            modules(SwerveModule::getDistanceFromCenter, Translation2d[]::new)
        );

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
    }

    /** Stop all swerve modules */
    public void stop() {
        modules(SwerveModule::stop);
    }

    /** Put all swerve modules to default state */
    public void toDefaultStates() {
        modules(SwerveModule::toDefaultState);
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

        this.desiredSpeeds = speeds;

        SwerveModule[] modules = modules(null, SwerveModule[]::new);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(desiredSpeeds);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(states[i]);
        }
    }

    /**
     * Set speeds of robot.
     * <li>vx: The velocity of the robot in the x (forward) direction in meter per second.</li>
     * <li>vy: The velocity of the robot in the y (sideways) direction in meter per second. (Positive values mean the robot is moving to the left).</li>
     * <li>omega: The angular velocity of the robot in radians per second.</li>
     * 
     * @see https://ibb.co/dJrL259
     * 
     * @param speeds Desired speeds of drivetrain (using swerve modules)
     * @param fieldRelative True if the robot is using a field relative coordinate system, false if using a robot relive coordinate system. If field relative, forward will be directly away from driver, no matter the rotation of the robot.
     * If robot relative, forward will be whatever direction the robot is facing in.
     */
    public void setDesiredState(ChassisSpeeds speeds, boolean fieldRelative) {

        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyro.getRotation2d());
        }

        setDesiredState(speeds);
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
     * @return was gyro able to zero yaw
     */
    public boolean zeroYaw() {
        if (gyro.isCalibrating()) return false;

        gyro.zeroYaw();

        return true;
    }

    // --- Util ---

    /**
     * Utility method. Function to easily run a function on each swerve module
     * 
     * @param func function to run on each swerve module, takes one argument and returns nothing, operates via side effects.
     */
    private void modules(Consumer<SwerveModule> func) {
        Stream.of(moduleFL, moduleFR, moduleBL, moduleBR).forEach(func);
    }

    /**
     * Utility method. Function to easily run a function on each swerve module and collect results to array.
     * Insures that we don't mix up order of swerve modules, as this could lead to hard to spot bugs.
     * 
     * @param <T> type that is returned by function and should be collected
     * @param func function that gets some data off each swerve module
     * @param arrayInitializer constructor function for array to collect results in, use T[]::new
     * @return array of results from func.
     */
    private <T> T[] modules(Function<? super SwerveModule, ? extends T> func, IntFunction<T[]> arrayInitializer) {
        Stream<SwerveModule> stream = Stream.of(moduleFL, moduleFR, moduleBL, moduleBR);

        if (func == null) return stream.toArray(arrayInitializer);
        return stream.map(func).toArray(arrayInitializer);
    }   
}
