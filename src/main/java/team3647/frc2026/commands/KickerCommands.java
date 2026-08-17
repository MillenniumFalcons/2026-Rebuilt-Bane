package team3647.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import team3647.frc2026.constants.KickerConstants;
import team3647.frc2026.subsystems.Kicker;

public class KickerCommands {
    Kicker kicker;

    public KickerCommands(Kicker kicker) {
        this.kicker = kicker;
    }

    public Command setOpenLoop(DoubleSupplier dutyCycle) {
        return Commands.run(() -> kicker.setOpenLoop(dutyCycle.getAsDouble()), kicker);
    }

    public Command preload() {
        return Commands.runEnd(
                () -> kicker.setVelocityFOC(KickerConstants.kLoadSpeed),
                () -> kicker.setOpenLoop(0),
                kicker);
    }

    public Command preloadFF() {
        return Commands.runEnd(
                () -> kicker.setVelocityFOC(KickerConstants.kLoadSpeed, 2.0),
                () -> kicker.setOpenLoop(0),
                kicker);
    }

    public Command spinRotations(double rotations) {
        return new Command() {
            double setpoint;

            @Override
            public void initialize() {
                setpoint = kicker.getPosition() + rotations;
                Logger.recordOutput("kicker setpoint", setpoint);
            }

            @Override
            public void execute() {
                kicker.setPosition(setpoint, 2.0);
            }

            @Override
            public boolean isFinished() {
                return Math.abs(kicker.getPosition() - setpoint) < 0.02;
            }
        };
    }

    public Command matchShooterSpeed(DoubleSupplier shooterSpeed) {
        return Commands.runEnd(() -> kicker.setVelocityFOC(4), () -> kicker.setOpenLoop(0), kicker);
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
