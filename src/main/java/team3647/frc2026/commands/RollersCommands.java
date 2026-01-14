package team3647.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2026.subsystems.Rollers;

public class RollersCommands {
    Rollers rollers;

    public RollersCommands(Rollers rollers) {
        this.rollers = rollers;
    }

    public Command setOpenLoop(double output) {
        return Commands.run(
                () -> {
                    rollers.setOpenLoop(output);
                },
                rollers);
    }

    public boolean currentGreater(double current) {
        return rollers.getMasterCurrent() >= current;
    }

    public Command kill() {
        return setOpenLoop(0).withTimeout(0.3);
    }

    public Command setVelocity(double vel) {
        return Commands.run(() -> rollers.setVelocity(vel, 0), rollers);
    }

    public boolean isAtVelocity(double velocity, double tolerance) {
        return Math.abs(rollers.getVelocity() - velocity) < tolerance;
    }

    public Command waitUntilAtVelocity(double velocity, double tolerance) {
        return Commands.waitUntil(() -> isAtVelocity(velocity, tolerance));
    }
}