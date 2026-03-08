package team3647.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2026.subsystems.Climb;

public class ClimbCommands {
    final Climb climb;

    public ClimbCommands(Climb climb) {
        this.climb = climb;
    }

    public Command up() {
        return Commands.runEnd(() -> climb.setOpenLoop(0.67), () -> climb.setOpenLoop(0), climb);
    }

    public Command down() {
        return Commands.run(() -> climb.setOpenLoop(-0.67), climb);
    }

    public Command kill() {
        return Commands.runOnce(() -> climb.setOpenLoop(0), climb);
    }
}
