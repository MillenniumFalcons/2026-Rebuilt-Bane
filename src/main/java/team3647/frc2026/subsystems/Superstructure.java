package team3647.frc2026.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2026.commands.KickerCommands;
import team3647.frc2026.commands.RollersCommands;

public class Superstructure {

    public final Rollers rollers;
    public final Kicker kicker;

    public final RollersCommands rollersCommands;
    public final KickerCommands kickerCommands;

    public Superstructure(
            Rollers rollers,
            Kicker kicker) {
        this.rollers = rollers;
        this.kicker = kicker;
        this.rollersCommands = new RollersCommands(this.rollers);
        this.kickerCommands = new KickerCommands(this.kicker);
    }

    public Command shoot(double velocity, double tolerance) {
        // Run roller control in parallel with the logic to wait and kick
        return Commands.parallel(
            rollersCommands.setVelocity(velocity),
            Commands.sequence(
                Commands.waitSeconds(1),
                kickerCommands.setOpenLoop(1)
            )
        );
    }
}
