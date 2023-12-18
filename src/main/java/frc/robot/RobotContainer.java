package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveDriveControl;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.SwerveModule;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
    private final ExampleCommand exampleCommand = new ExampleCommand(exampleSubsystem);

    private final SwerveModule swerveModuleFL = new SwerveModule(
			SwerveModuleConstants.ANGULAR_MOTOR_ID_FL,
			SwerveModuleConstants.VELOCITY_MOTOR_ID_FL,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_FL,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_FL);
	private final SwerveModule swerveModuleFR = new SwerveModule(
			SwerveModuleConstants.ANGULAR_MOTOR_ID_FR,
			SwerveModuleConstants.VELOCITY_MOTOR_ID_FR,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_FR,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_FR);
	private final SwerveModule swerveModuleBL = new SwerveModule(
			SwerveModuleConstants.ANGULAR_MOTOR_ID_BL,
			SwerveModuleConstants.VELOCITY_MOTOR_ID_BL,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_BL,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_BL);
	private final SwerveModule swerveModuleBR = new SwerveModule(
			SwerveModuleConstants.ANGULAR_MOTOR_ID_BR,
			SwerveModuleConstants.VELOCITY_MOTOR_ID_BR,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_ID_BR,
			SwerveModuleConstants.ANGULAR_MOTOR_ENCODER_OFFSET_BR);

	private final AHRS gyro = new AHRS(I2C.Port.kMXP);

    // Create joysticks
    private final CommandJoystick driverJoystick = new CommandJoystick(DriverConstants.DRIVER_JOYSTICK_PORT);
    // private final CommandJoystick operatorJoystick = new CommandJoystick(OperatorConstants.OPERATOR_JOYSTICK_PORT);

	private final SwerveDrivetrain drivetrain = new SwerveDrivetrain(gyro, swerveModuleFL, swerveModuleFR, swerveModuleBL, swerveModuleBR);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
		drivetrain.setDefaultCommand(new SwerveDriveControl(drivetrain, driverJoystick));

        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings.
     */
    private void configureBindings() {
        driverJoystick.button(1).onTrue(exampleCommand);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return Autos.exampleAuto(exampleSubsystem);
    }
}
