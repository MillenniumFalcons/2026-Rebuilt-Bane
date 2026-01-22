package team3647.frc2026.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import team3647.frc2026.commands.HoodCommands;
import team3647.frc2026.commands.KickerCommands;
import team3647.frc2026.commands.ShooterCommands;

public class Superstructure {

    public final ShooterLeft shooterOne;
    public final ShooterRight shooterTwo;
    public final Kicker kicker;
    public final Hood hood;

    public final ShooterCommands shooterCommands;
    public final KickerCommands kickerCommands;
    public final HoodCommands hoodCommands;

    public final DoubleSupplier hoodAngle;

    public Superstructure(
            ShooterLeft shooterOne,
            ShooterRight shooterTwo,
            Kicker kicker,
            Hood hood,
            DoubleSupplier hoodAngle) {
        this.shooterOne = shooterOne;
        this.shooterTwo = shooterTwo;
        this.hood = hood;
        this.kicker = kicker;
        this.hoodAngle = hoodAngle;
        this.shooterCommands = new ShooterCommands(this.shooterOne, this.shooterTwo);
        this.kickerCommands = new KickerCommands(this.kicker);
        this.hoodCommands = new HoodCommands(hood);
    }

    public Command prep() {
        return hoodCommands.setAngleDeg(hoodAngle);
    }

    public Command shoot(double velocity, double tolerance) {
        // fix the wait time thingy
        return Commands.parallel(
                shooterCommands.setVelocity(velocity),
                Commands.sequence(Commands.waitSeconds(1), kickerCommands.setOpenLoop(1)));
    }
}
