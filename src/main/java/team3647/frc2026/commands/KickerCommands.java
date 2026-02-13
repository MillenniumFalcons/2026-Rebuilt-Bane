package team3647.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2026.subsystems.Kicker;

public class KickerCommands {
    Kicker kicker;

    public KickerCommands(Kicker kicker) {
        this.kicker = kicker;
    }

    public Command kick() {
        return Commands.run(() -> kicker.setOpenLoop(0.8), kicker);
    }

    public boolean currentGreater(double current) {
        return kicker.getMasterCurrent() >= current;
    }

    public Command kill() {
        return Commands.runOnce(() -> kicker.setOpenLoop(0), kicker);
    }
}
