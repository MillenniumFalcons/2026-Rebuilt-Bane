package team3647.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import team3647.frc2026.subsystems.Rollers;

public class RollerCommands {
    final Rollers rollers;
    private DoubleSupplier vxSupplier = () -> 0.0;

    // TunableNumber intakeSpeedGain = new TunableNumber("intake speed gain", 4);

    public RollerCommands(Rollers rollers) {
        this.rollers = rollers;
    }

    public void setVxSupplier(DoubleSupplier vxSupplier) {
        this.vxSupplier = vxSupplier;
    }

    public Command intake() {
        return Commands.run(() -> rollers.setOpenLoop(0.6), rollers);
        // return Commands.runEnd(
        //         () ->
        //                 rollers.setVelocityTorque(
        //                         RollerConstants.kIntakeBaseSpeed
        //                                 + intakeSpeedGain.get()
        //                                         * Math.max(0, vxSupplier.getAsDouble())),
        //         () -> rollers.setOpenLoop(0),
        //         rollers);
    }

    public Command intakeFast() {
        return Commands.run(() -> rollers.setOpenLoop(0.8), rollers);
    }

    public Command jerkOff() {
        return Commands.sequence(
                        Commands.run(() -> rollers.setOpenLoop(0.5), rollers).withTimeout(0.05),
                        Commands.run(() -> rollers.setOpenLoop(-0.5), rollers).withTimeout(0.05))
                .repeatedly();
    }

    public Command spit() {
        return Commands.run(() -> rollers.setOpenLoop(-0.6), rollers);
    }

    public boolean currentGreater(double current) {
        return rollers.getMasterCurrent() >= current;
    }

    public Command kill() {
        return Commands.runOnce(() -> rollers.setOpenLoop(0), rollers);
    }
}
