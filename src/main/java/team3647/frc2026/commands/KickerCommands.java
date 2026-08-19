package team3647.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import java.util.function.DoubleSupplier;
import team3647.frc2026.subsystems.Kicker;
import team3647.lib.tunables.TunableDouble;

public class KickerCommands {
    Kicker kicker;
    double setpoint;
    Double speed = 4.0;
    TunableDouble tunable = new TunableDouble("kicker speed", speed);

    public KickerCommands(Kicker kicker) {
        this.kicker = kicker;
    }

    public Command setOpenLoop(DoubleSupplier dutyCycle) {
        return Commands.run(() -> kicker.setOpenLoop(dutyCycle.getAsDouble()), kicker);
    }

    public Command preload() {
        return Commands.runEnd(
                () -> kicker.setVelocityFOC(0.4), () -> kicker.setOpenLoop(0), kicker);
    }

    public Command spinRotations(double rotations) {
        return new FunctionalCommand(
                () -> setpoint = kicker.getPosition() + rotations,
                () -> kicker.setPosition(setpoint, 2.0),
                (interrupted) -> {
                    if (!interrupted) {
                        kicker.setOpenLoop(0);
                    }
                },
                () -> Math.abs(kicker.getPosition() - setpoint) < 0.02,
                kicker);
    }

    public Command matchShooterSpeed(DoubleSupplier shooterSpeed) {
        return Commands.runEnd(
                () -> {
                    kicker.setVelocityFOC(tunable.get());
                },
                () -> kicker.setOpenLoop(0),
                kicker);
    }

    public Command reverse() {
        return Commands.run(() -> kicker.setVelocityFOC(-5), kicker);
    }

    public boolean currentGreater(double current) {
        return kicker.getMasterCurrent() >= current;
    }

    public Command kill() {
        return Commands.runOnce(() -> kicker.setOpenLoop(0), kicker);
    }
}
