package team3647.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import team3647.frc2026.subsystems.Rollers;

public class RollerCommands {
    final Rollers rollers;

    public RollerCommands(Rollers rollers) {
        this.rollers = rollers;
    }

    public Command intakeFast() {
        return Commands.runEnd(
                () -> rollers.setOpenLoop(1.0), () -> rollers.setOpenLoop(0), rollers);
    }

    public Command intake() {
        return Commands.runEnd(
                () -> rollers.setOpenLoop(1.0), () -> rollers.setOpenLoop(0), rollers);
    }

    public Command jerkOff() {
        return Commands.sequence(
                        Commands.run(() -> rollers.setOpenLoop(0.5), rollers).withTimeout(0.05),
                        Commands.run(() -> rollers.setOpenLoop(-0.5), rollers).withTimeout(0.05))
                .repeatedly();
    }

    public Command spit() {
        return Commands.run(() -> rollers.setOpenLoop(-1.0), rollers);
    }

    public boolean currentGreater(double current) {
        return rollers.getMasterCurrent() >= current;
    }

    public Command kill() {
        return Commands.runOnce(() -> rollers.setOpenLoop(0), rollers);
    }
}
