package team3647.frc2026.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2026.subsystems.Rollers;

public class RollerCommands {
    final Rollers rollers;

    public RollerCommands(Rollers rollers) {
        this.rollers = rollers;
    }

    public Command intakeFast() {
        return Commands.runEnd(
                () -> rollers.setOpenLoop(0.85), () -> rollers.setOpenLoop(0), rollers);
    }

    public Command intake() {
        return Commands.runEnd(
                () -> rollers.setOpenLoop(0.7), () -> rollers.setOpenLoop(0), rollers);
    }

    public Command intakeSpeedScale(DoubleSupplier robotSpeed) {
        return Commands.run(() -> rollers.setVelocityMoving(robotSpeed.getAsDouble()), rollers);
    }

    public Command spit() {
        return Commands.run(() -> rollers.setOpenLoop(-0.85), rollers);
    }

    public boolean currentGreater(double current) {
        return rollers.getMasterCurrent() >= current;
    }

    public Command kill() {
        return Commands.runOnce(() -> rollers.setOpenLoop(0), rollers);
    }
}
