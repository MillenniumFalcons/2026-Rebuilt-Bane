package team3647.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import team3647.frc2026.subsystems.Kicker;

public class KickerCommands {
    Kicker kicker;

    public KickerCommands(Kicker kicker) {
        this.kicker = kicker;
    }

    public Command kick() {
        return Commands.runEnd(() -> kicker.setOpenLoop(0.25), () -> kicker.setOpenLoop(0), kicker);
    }

    public Command matchShooterSpeed(DoubleSupplier shooterSpeed) {
        return Commands.runEnd(() -> kicker.setVelocityFOC(4), () -> kicker.setOpenLoop(0), kicker);
    }

    public Command reverse() {
        return Commands.run(() -> kicker.setOpenLoop(-0.75), kicker);
    }

    public Command jamProtect() {
        return Commands.run(() -> kicker.setOpenLoop(-0.25), kicker);
    }

    public boolean currentGreater(double current) {
        return kicker.getMasterCurrent() >= current;
    }

    public Command kill() {
        return Commands.runOnce(() -> kicker.setOpenLoop(0), kicker);
    }
}
