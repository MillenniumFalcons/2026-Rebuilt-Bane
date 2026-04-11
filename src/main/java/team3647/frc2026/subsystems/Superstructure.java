package team3647.frc2026.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import team3647.frc2026.commands.HoodCommands;
import team3647.frc2026.commands.IndexerCommands;
import team3647.frc2026.commands.KickerCommands;
import team3647.frc2026.commands.RollerCommands;
import team3647.frc2026.commands.ShooterCommands;
import team3647.frc2026.commands.WristCommands;
import team3647.frc2026.util.LEDStates;
import team3647.frc2026.util.LEDStates.SuperstructureState;
import team3647.frc2026.util.RobotTracker;

public class Superstructure {

    public final Shooter shooter;
    public final Indexer indexer;
    public final Kicker kicker;
    public final Hood hood;
    public final Wrist wrist;
    public final Rollers rollers;

    public final ShooterCommands shooterCommands;
    public final IndexerCommands indexerCommands;
    public final KickerCommands kickerCommands;
    public final HoodCommands hoodCommands;
    public final WristCommands wristCommands;
    public final RollerCommands rollerCommands;

    public final DoubleSupplier hoodAngle;
    public final DoubleSupplier shotSpeed;
    public Angle wantedIntakeAngle = Degrees.of(70);
    public final BooleanSupplier isRotAligned;

    public final RobotTracker robotTracker;

    public final Trigger atShotVel;
    public final BooleanSupplier atHoodAngle;
    public final Trigger readyToShoot;

    public final LEDStates ledStates;

    public Superstructure(
            Shooter shooter,
            Indexer indexer,
            Hood hood,
            Kicker kicker,
            Wrist wrist,
            Rollers rollers,
            LEDStates ledStates,
            RobotTracker robotTracker,
            DoubleSupplier hoodAngle,
            DoubleSupplier shotSpeed,
            BooleanSupplier isRotAligned) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.hood = hood;
        this.kicker = kicker;
        this.wrist = wrist;
        this.rollers = rollers;
        this.robotTracker = robotTracker;
        this.hoodAngle = hoodAngle;
        this.shotSpeed = shotSpeed;
        this.isRotAligned = isRotAligned;

        this.shooterCommands = new ShooterCommands(this.shooter);
        this.indexerCommands = new IndexerCommands(this.indexer);
        this.kickerCommands = new KickerCommands(this.kicker);
        this.hoodCommands = new HoodCommands(this.hood);
        this.wristCommands = new WristCommands(this.wrist);
        this.rollerCommands = new RollerCommands(this.rollers);
        this.rollerCommands.setVxSupplier(this.robotTracker::getIntakeDirectionSpeed);

        this.atShotVel = new Trigger(() -> shooterCommands.isAtVelocity(this.shotSpeed, 0.02));
        this.atHoodAngle = () -> hoodCommands.angleReached(this.hoodAngle, 2.0);
        this.readyToShoot = new Trigger(this.isRotAligned).debounce(0.1);

        this.ledStates = ledStates;
    }

    public LEDStates getStates() {
        return ledStates;
    }

    public Angle getIntakeAngle() {
        return wantedIntakeAngle;
    }

    public Command feed() {
        return Commands.sequence(
                Commands.waitUntil(readyToShoot), Commands.parallel(deployIntake(), reverse()));
    }

    public Command logEverything() {
        return Commands.parallel(
                Commands.run(
                        () ->
                                Logger.recordOutput(
                                        "Shooting/ShotSpeed/wantedshotspeed",
                                        shotSpeed.getAsDouble())),
                Commands.run(
                        () ->
                                Logger.recordOutput(
                                        "Shooting/ShotSpeed/shootervelocity",
                                        shooter.getVelocity())),
                Commands.run(
                        () ->
                                Logger.recordOutput(
                                        "Shooting/ShotSpeed/atshotvel", atShotVel.getAsBoolean())),
                Commands.run(
                        () ->
                                Logger.recordOutput(
                                        "Shooting/readytoshoot", readyToShoot.getAsBoolean())),
                Commands.run(
                        () ->
                                Logger.recordOutput(
                                        "Shooting/Hood/athoodangle", atHoodAngle.getAsBoolean())),
                Commands.run(
                        () ->
                                Logger.recordOutput(
                                        "Shooting/Hood/wantedhoodeangle", hoodAngle.getAsDouble())),
                Commands.run(
                        () ->
                                Logger.recordOutput(
                                        "Shooting/Hood/currenthoodangle", hood.getPosition())),
                Commands.run(
                        () ->
                                Logger.recordOutput(
                                        "Shooting/Rotation/isrotaligned",
                                        isRotAligned.getAsBoolean())));
    }

    public Command shoot() {
        return Commands.parallel(logEverything(), prep(), indexWhenReady(), jerkOff());
    }

    public Command prep() {
        return shooterCommands
                .setVelocity(shotSpeed)
                .deadlineFor(ledStates.activateState(SuperstructureState.PREPPING));
    }

    public Command bumpShot() {
        return Commands.parallel(logEverything(), prepForBump(), index());
    }

    public Command prepForBump() {
        return shooterCommands
                .setVelocity(() -> 8.5)
                .deadlineFor(ledStates.activateState(SuperstructureState.PREPPING));
    }

    public Command indexWhenReady() {
        return Commands.sequence(
                Commands.waitUntil(readyToShoot.and(atShotVel).and(atHoodAngle)),
                index()
                        // .onlyWhile(readyToShoot)
                        //         .repeatedly()
                        .deadlineFor(ledStates.activateState(SuperstructureState.SHOOTING)));
    }

    public Command index() {
        return Commands.parallel(
                indexerCommands.index(), kickerCommands.matchShooterSpeed(shotSpeed));
    }

    public Command reverse() {
        return Commands.parallel(
                indexerCommands.reverse(), kickerCommands.reverse(), rollerCommands.spit());
    }

    public Command intake() {
        return Commands.parallel(wristCommands.deploy(), rollerCommands.intake())
                .deadlineFor(ledStates.activateState(SuperstructureState.INTAKING));
    }

    public Command intakeWithDelay() {
        return Commands.sequence(
                        Commands.waitSeconds(0.5),
                        Commands.parallel(wristCommands.deploy(), rollerCommands.intake()))
                .deadlineFor(ledStates.activateState(SuperstructureState.INTAKING));
    }

    public Command retractIntake() {
        return Commands.parallel(wristCommands.retract());
    }

    public Command deployIntake() {
        return Commands.parallel(wristCommands.deploy());
    }

    public Command spinUpTest() {
        return Commands.parallel(shooterCommands.setOpenLoop(1));
    }

    public Command prepForInterp() {
        return shooterCommands.setVelocityTunable();
    }

    public Command interpShot() {
        return Commands.parallel(
                prepForInterp(),
                hoodCommands.setInterpAngle(),
                Commands.sequence(Commands.waitSeconds(4.0), index()));
    }

    public Command jerkOff() {
        return Commands.parallel(
                        Commands.sequence(
                                deployIntake().withTimeout(0.5), retractIntake().withTimeout(0.5)),
                        rollerCommands.intake().withTimeout(1))
                .repeatedly();
    }
}
