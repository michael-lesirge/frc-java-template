package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.OptionButton;
import frc.robot.utils.OptionButton.ActivationMode;

public class FindSwerveOffset extends CommandBase {

    private final SwerveModule module;
    private final CommandJoystick joystick;

    static private final double speed = 0.3;
    private double degrees = 0;

    OptionButton plusButton;
    OptionButton minusButton;

    OptionButton holdPlusButton;
    OptionButton holdMinusButton;

    public FindSwerveOffset(SwerveModule module, CommandJoystick joystick) {

        this.module = module;
        this.joystick = joystick;

        plusButton = new OptionButton(joystick, 7, ActivationMode.TAP);
        minusButton = new OptionButton(joystick, 8, ActivationMode.TAP);

        holdPlusButton = new OptionButton(joystick, 9, ActivationMode.HOLD);
        holdMinusButton = new OptionButton(joystick, 10, ActivationMode.HOLD);
        
        addRequirements(module);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("Module Speed M/S", speed);
    }

    @Override
    public void execute() {

        final double speedCoefficient = joystick.button(1).getAsBoolean() ? 1 : 10;

        final double baseTapSpeed = 1;
        final double baseHoldSpeed = 5;

        if (plusButton.getState()) {
			degrees += baseTapSpeed * speedCoefficient;
		} else if (minusButton.getState()) {
			degrees -= baseTapSpeed * speedCoefficient;
		} else if (holdPlusButton.getState()) {
			degrees += baseHoldSpeed * speedCoefficient;
		} else if (holdMinusButton.getState()) {
			degrees -= baseHoldSpeed * speedCoefficient;
		}

        SmartDashboard.putNumber("Module Rotation Degrees (Target)", degrees);
        
        SwerveModuleState targetState = new SwerveModuleState(speed, Rotation2d.fromDegrees(degrees));

        module.setDesiredState(
            targetState, false
        );

        SwerveModuleState realState = module.getState();
        
        SmartDashboard.putNumber("Module Rotation Degrees (Real)", realState.angle.getDegrees());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        module.stop();
    }
}
