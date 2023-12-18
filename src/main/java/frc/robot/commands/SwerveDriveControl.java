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
    public SwerveDriveControl(SwerveDrivetrain drivetrain, CommandJoystick joystick) {
        
        // save parameters
        this.drivetrain = drivetrain;
        this.joystick = joystick;

        // Create and configure buttons
        preciseModeButton = new OptionButton(joystick, 1, ActivationMode.HOLD);
        boostModeButton = new OptionButton(joystick, 0, ActivationMode.TOGGLE);
        fieldRelieveButton = new OptionButton(joystick, 2, ActivationMode.TOGGLE);

        // Tell the command schedular we are using the drivetrain
        addRequirements(drivetrain);
    }

    /**
     * The initial subroutine of a command. Called once when the command is initially scheduled.
     * Put all swerve modules to default state, staying still and face forwards.
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
        double speedX = applyJoystickDeadzone(-joystick.getX());
		double speedY = applyJoystickDeadzone(-joystick.getY());
		double speedOmega = applyJoystickDeadzone(joystick.getTwist());

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

        // Level of speed from Precise, Normal, Boost

        // find our speed level, default is one
        final int speedLevel = 1
            - preciseModeButton.getStateInt()
            + boostModeButton.getStateInt();

        // for testing
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
     * @param joystickValue Value between -1 and 1 from joystick  X or Y axis
     * @return 0 if joystickValue is in deadzone, else the value scaled to the new control area
     */
    private static double applyJoystickDeadzone(double joystickValue) {

        final double deadzone = DriverConstants.JOYSTICK_DEAD_ZONE;

        if (Math.abs(joystickValue) <= deadzone) {
            // If the joystick |value| is in the deadzone than zero it out
            joystickValue = 0;
        }
        else {
            // if not than just move the value closer to zero by deadzone so that we don't just from 0 to 0.2 (or whatever deadzone is)
            joystickValue -= deadzone * (joystickValue / Math.abs(joystickValue));
        }

        // scale value back up to account for subtraction
        joystickValue *= 1 + deadzone;

        return joystickValue;
    }
}