package team3647.frc2026.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import team3647.frc2026.commands.HoodCommands;
import team3647.frc2026.commands.IndexerCommands;
import team3647.frc2026.commands.KickerCommands;
import team3647.frc2026.commands.ShooterCommands;

public class Superstructure {

    public final ShooterLeft shooterOne;
    public final ShooterRight shooterTwo;
    public final Indexer indexer;
    public final Kicker kicker;
    public final Hood hood;

    public final ShooterCommands shooterCommands;
    public final IndexerCommands indexerCommands;
    public final KickerCommands kickerCommands;
    public final HoodCommands hoodCommands;

    public final DoubleSupplier hoodAngle;
    public final DoubleSupplier shotSpeed;
    public Angle wantedIntakeAngle = Degrees.of(70);
    public final BooleanSupplier isRotAligned;

    public Superstructure(
            ShooterLeft shooterOne,
            ShooterRight shooterTwo,
            Indexer indexer,
            Hood hood,
            Kicker kicker,
            DoubleSupplier hoodAngle,
            DoubleSupplier shotSpeed,
            BooleanSupplier isRotAligned) {
        this.shooterOne = shooterOne;
        this.shooterTwo = shooterTwo;
        this.hood = hood;
        this.indexer = indexer;
        this.kicker = kicker;
        this.hoodAngle = hoodAngle;
        this.shooterCommands = new ShooterCommands(this.shooterOne, this.shooterTwo);
        this.indexerCommands = new IndexerCommands(this.indexer);
        this.kickerCommands = new KickerCommands(this.kicker);
        this.hoodCommands = new HoodCommands(hood);
        this.shotSpeed = shotSpeed;
        this.isRotAligned = isRotAligned;
    }

    public Angle getIntakeAngle() {
        return wantedIntakeAngle;
    }

    public Command prep() {
        return hoodCommands.setAngleDeg(hoodAngle);
    }

    public Command shoot() {

        // this one is for tuning
        // return Commands.parallel(
        //         shooterCommands.shootTunable(),
        //         Commands.sequence(Commands.waitSeconds(1), index()));
        return Commands.parallel(
                shooterCommands.setVelocity(shotSpeed),
                Commands.run(() -> Logger.recordOutput("wanted speed", shotSpeed.getAsDouble())),
                Commands.sequence(
                        shooterCommands.waitUntilAtVelocity(shotSpeed, 0.5),
                        Commands.waitUntil(isRotAligned),
                        index()));
    }

    public Command shotTuning() {

        // fix the wait time thingy
        // return Commands.parallel(
        //         shooterCommands.shoot(),
        //         Commands.sequence(Commands.waitSeconds(1), index()));
        return Commands.parallel(
                shooterCommands.setVelocity(shotSpeed),
                Commands.run(() -> Logger.recordOutput("wanted speed", shotSpeed.getAsDouble())));
    }

    public Command index() {
        return Commands.parallel(indexerCommands.index(), kickerCommands.kick());
    }

    public Command reverse() {
        return Commands.parallel(indexerCommands.reverse(), kickerCommands.reverse());
    }
}
