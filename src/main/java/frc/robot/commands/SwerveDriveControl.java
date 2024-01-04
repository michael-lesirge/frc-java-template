package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.DriverConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.OptionButton;
import frc.robot.utils.OptionButton.ActivationMode;

/**
 * This is the default command for the drivetrain, allowing for remote operation with joystick
 */
public class SwerveDriveControl extends CommandBase {
    private final SwerveDrivetrain drivetrain;
    private final CommandJoystick joystick;

    private OptionButton preciseModeButton;
    private OptionButton boostModeButton;
    private OptionButton fieldRelieveButton;

    /**
	 * Creates a new SwerveDriveControl Command.
	 *
	 * @param drivetrain The drivetrain of the robot
	 * @param joystick The joystick used to control drivetrain
	 */
    public SwerveDriveControl(SwerveDrivetrain drivetrain, CommandJoystick driverJoystick) {
        
        // save parameters
        this.drivetrain = drivetrain;
        this.joystick = driverJoystick;

        // Create and configure buttons
        preciseModeButton = new OptionButton(joystick, 2, ActivationMode.TOGGLE);
        boostModeButton = new OptionButton(joystick, 1, ActivationMode.HOLD);
        fieldRelieveButton = new OptionButton(joystick, 3, ActivationMode.TOGGLE);

        // Tell the command schedular we are using the drivetrain
        addRequirements(drivetrain);
    }

    /**
     * The initial subroutine of a command. Called once when the command is initially scheduled.
     * Puts all swerve modules to the default state, staying still and facing forwards.
     */
    @Override
    public void initialize() {
        drivetrain.toDefaultStates();
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled (Every 20 ms).
     */
    @Override
    public void execute() {

        // Get joystick inputs
        final double speedX = applyJoystickDeadzone(-joystick.getX(), DriverConstants.JOYSTICK_DEAD_ZONE);
		final double speedY = applyJoystickDeadzone(-joystick.getY(), DriverConstants.JOYSTICK_DEAD_ZONE);
		final double speedOmega = applyJoystickDeadzone(joystick.getTwist(), DriverConstants.JOYSTICK_DEAD_ZONE);

        // // Code for rotating with buttons if driver prefers 
        // double speedOmega = 0;
        // if (joystick.button(7).getAsBoolean()) {
		// 	speedOmega += OperatorConstants.maxSpeedOptionsRotation[0];
		// } else if (joystick.button(8).getAsBoolean()) {
		// 	speedOmega -= OperatorConstants.maxSpeedOptionsRotation[0];
		// } else if (joystick.button(9).getAsBoolean()) {
		// 	speedOmega += OperatorConstants.maxSpeedOptionsRotation[1];
		// } else if (joystick.button(10).getAsBoolean()) {
		// 	speedOmega -= OperatorConstants.maxSpeedOptionsRotation[1];
		// }

        
        // Level of speed from Precise, to Normal, to Boost
        // Find our speed level, default is one (Normal)
        final int speedLevel = 1
            - preciseModeButton.getStateInt()
            + boostModeButton.getStateInt();

        // Can be changed for testing
        final int speedCoefficient = 1;

        final boolean isFieldRelative = fieldRelieveButton.getState();

        final ChassisSpeeds speeds = new ChassisSpeeds(
            speedX * DriverConstants.maxSpeedOptionsTranslation[speedLevel] * speedCoefficient,
            speedY * DriverConstants.maxSpeedOptionsTranslation[speedLevel] * speedCoefficient,
            speedOmega * DriverConstants.maxSpeedOptionsRotation[speedLevel] * speedCoefficient
        );

        // Display relevant data on shuffleboard.
        SmartDashboard.putString("Speed Mode", DriverConstants.maxSpeedOptionsNames[speedLevel]);        
        SmartDashboard.putBoolean("Field Relieve", isFieldRelative);

        SmartDashboard.putNumber("SpeedsX", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("SpeedsY", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("SpeedsOmega", speeds.omegaRadiansPerSecond);

        final ChassisSpeeds realSpeeds = drivetrain.getState();

        SmartDashboard.putNumber("Real SpeedsX", realSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Real SpeedsY", realSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Real SpeedsOmega", realSpeeds.omegaRadiansPerSecond);

        drivetrain.setDesiredState(speeds, isFieldRelative);
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end() method and un-schedule it.
     * Always return false since we never want to end in this case
     */
	@Override
	public boolean isFinished() {
		return false;
	}

    /**
     * The action to take when the command ends. Called when either the command finishes normally, or when it interrupted/canceled.
     * Should only happen in this case if we get interrupted
     */
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    // --- Util ---

    /**
     * Utility method. Apply a deadzone to the joystick output to account for stick drift and small bumps.
     * 
     * @param joystickValue Value in [-1, 1] from joystick axis
     * @return {@code 0} if {@code |joystickValue| <= deadzone}, else the {@code joystickValue} scaled to the new control area
     */
    private static double applyJoystickDeadzone(double joystickValue, double deadzone) {
        if (Math.abs(joystickValue) <= deadzone) {
            // If the joystick |value| is in the deadzone than zero it out
            return 0;
        }

        // scale value from the range [0, 1] to (deadzone, 1]
        return joystickValue * (1 + deadzone) - Math.signum(joystickValue) * deadzone;
    }
}