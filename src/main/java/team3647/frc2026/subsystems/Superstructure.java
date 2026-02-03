package team3647.frc2026.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import team3647.frc2026.commands.HoodCommands;
import team3647.frc2026.commands.IndexerCommands;
import team3647.frc2026.commands.ShooterCommands;

public class Superstructure {

    public final ShooterLeft shooterOne;
    public final ShooterRight shooterTwo;
    public final Indexer indexer;
    public final Hood hood;

    public final ShooterCommands shooterCommands;
    public final IndexerCommands indexerCommands;
    public final HoodCommands hoodCommands;

    public final DoubleSupplier hoodAngle;
    public final DoubleSupplier shotSpeed;

    public Superstructure(
            ShooterLeft shooterOne,
            ShooterRight shooterTwo,
            Indexer indexer,
            Hood hood,
            DoubleSupplier hoodAngle,
            DoubleSupplier shotSpeed) {
        this.shooterOne = shooterOne;
        this.shooterTwo = shooterTwo;
        this.hood = hood;
        this.indexer = indexer;
        this.hoodAngle = hoodAngle;
        this.shooterCommands = new ShooterCommands(this.shooterOne, this.shooterTwo);
        this.indexerCommands = new IndexerCommands(this.indexer);
        this.hoodCommands = new HoodCommands(hood);
        this.shotSpeed = shotSpeed;
    }

    public Command prep() {
        return hoodCommands.setAngleDeg(hoodAngle);
    }

    public Command shoot(double velocity, double tolerance) {
        // fix the wait time thingy
        return Commands.parallel(shooterCommands.setVelocity(velocity));
    }

    public Command index(double percentage) {
        return indexerCommands.setOpenLoop(percentage);
    }
}
