package frc.robot.commands;

import frc.robot.subsystems.TemplateSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
    /** Example static factory for an autonomous command. */
    public static CommandBase exampleAuto(TemplateSubsystem subsystem) {
        return Commands.sequence(
                new TemplateCommand(subsystem));
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
