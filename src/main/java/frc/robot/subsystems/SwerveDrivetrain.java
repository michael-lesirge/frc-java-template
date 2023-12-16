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

    private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);

    private final SwerveDriveOdometry odometry;

    private ChassisSpeeds desiredSpeeds;

    private boolean isFieldRelative = false;

    private final SwerveModule moduleFL;
    private final SwerveModule moduleFR;
    private final SwerveModule moduleBL;
    private final SwerveModule moduleBR;

    private final Stream<SwerveModule> allModules;

    private final AHRS gyro;

    private Pose2d pose = new Pose2d();

    public SwerveDrivetrain(AHRS gyro, SwerveModule swerveModuleFL, SwerveModule swerveModuleFR, SwerveModule swerveModuleBL, SwerveModule swerveModuleBR) {
        this.gyro = gyro;
        moduleFL = swerveModuleFL;
        moduleFR = swerveModuleFR;
        moduleBL = swerveModuleBL;
        moduleBR = swerveModuleBR;
        allModules = Stream.of(moduleFL, moduleFR, moduleBL, moduleBR);

        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(),
                modules(SwerveModule::getPosition, SwerveModulePosition[]::new));
    }

    @Override
    public void periodic() {
        pose = odometry.update(gyro.getRotation2d(), modules(SwerveModule::getPosition, SwerveModulePosition[]::new));

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
     * <li>vx: The velocity of the robot in the x (forward) direction in meter per
     * second.</li>
     * <li>vy: The velocity of the robot in the y (sideways) direction in meter per
     * second. (Positive values mean the robot is moving to the left).</li>
     * <li>omega: The angular velocity of the robot in radians per second.</li>
     * 
     * @return Speeds of drivetrain (from swerve modules)
     */
    public ChassisSpeeds getState() {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(modules(SwerveModule::getState, SwerveModuleState[]::new));

        if (isFieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, gyro.getRotation2d());
        }

        return speeds;
    }

    /**
     * Set speeds of robot.
     * <li>vx: The velocity of the robot in the x (forward) direction in meter per
     * second.</li>
     * <li>vy: The velocity of the robot in the y (sideways) direction in meter per
     * second. (Positive values mean the robot is moving to the left).</li>
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
        odometry.resetPosition(gyro.getRotation2d(), modules(SwerveModule::getPosition, SwerveModulePosition[]::new),
                pose);
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

    private void modules(Consumer<SwerveModule> func) {
        allModules.forEach(func);
    }

    private <T> T[] modules(Function<SwerveModule, T> func, IntFunction<T[]> arrayInitializer) {
        if (func == null)
            return allModules.toArray(arrayInitializer);
        return allModules.map(func).toArray(arrayInitializer);
    }
}
