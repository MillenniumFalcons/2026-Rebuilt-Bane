package team3647.frc2026.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2026.commands.KickerCommands;
import team3647.frc2026.commands.ShooterCommands;

public class Superstructure {

    public final ShooterOne shooterOne;
    public final ShooterTwo shooterTwo;
    public final Kicker kicker;

    public final ShooterCommands shooterCommands;
    public final KickerCommands kickerCommands;

    public Superstructure(
            ShooterOne shooterOne,
            ShooterTwo shooterTwo,
            Kicker kicker) {
        this.shooterOne = shooterOne;
        this.shooterTwo = shooterTwo;
        this.kicker = kicker;
        this.shooterCommands = new ShooterCommands(this.shooterOne, this.shooterTwo);
        this.kickerCommands = new KickerCommands(this.kicker);
    }

    public Command shoot(double velocity, double tolerance) {
        // Run roller control in parallel with the logic to wait and kick
        return Commands.parallel(
            shooterCommands.setVelocity(velocity),
            Commands.sequence(
                Commands.waitSeconds(1),
                kickerCommands.setOpenLoop(1)
            )
        );
    }
}
