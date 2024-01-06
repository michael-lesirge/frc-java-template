package frc.robot;

import frc.robot.Constants.DriverConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.TemplateCommand;
import frc.robot.subsystems.TemplateSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

/**
 * This class is where the bulk of the robot should be declared.
 * Since Command-based is a "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final TemplateSubsystem exampleSubsystem = new TemplateSubsystem();
    private final TemplateCommand exampleCommand = new TemplateCommand(exampleSubsystem);


    // Create joysticks
    private final CommandJoystick driverJoystick = new CommandJoystick(DriverConstants.DRIVER_JOYSTICK_PORT);
    // private final CommandJoystick operatorJoystick = new CommandJoystick(OperatorConstants.OPERATOR_JOYSTICK_PORT);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // Configure the trigger bindings
        configureBindings();
    }

    /** Use this method to define your trigger->command mappings. */
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
