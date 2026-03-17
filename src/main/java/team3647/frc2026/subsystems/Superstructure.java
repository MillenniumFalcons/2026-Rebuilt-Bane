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
import team3647.frc2026.util.RobotTracker;

public class Superstructure {

    public final ShooterLeft shooterOne;
    public final ShooterRight shooterTwo;
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

    public final BooleanSupplier atShotVel;
    public final BooleanSupplier atHoodAngle;
    public final Trigger readyToShoot;

    public SuperstructureState currentState = SuperstructureState.NONE;

    public Superstructure(
            ShooterLeft shooterOne,
            ShooterRight shooterTwo,
            Indexer indexer,
            Hood hood,
            Kicker kicker,
            Wrist wrist,
            Rollers rollers,
            RobotTracker robotTracker,
            DoubleSupplier hoodAngle,
            DoubleSupplier shotSpeed,
            BooleanSupplier isRotAligned) {
        this.shooterOne = shooterOne;
        this.shooterTwo = shooterTwo;
        this.indexer = indexer;
        this.hood = hood;
        this.kicker = kicker;
        this.wrist = wrist;
        this.rollers = rollers;
        this.robotTracker = robotTracker;
        this.hoodAngle = hoodAngle;
        this.shotSpeed = shotSpeed;
        this.isRotAligned = isRotAligned;

        this.shooterCommands = new ShooterCommands(this.shooterOne, this.shooterTwo);
        this.indexerCommands = new IndexerCommands(this.indexer);
        this.kickerCommands = new KickerCommands(this.kicker);
        this.hoodCommands = new HoodCommands(this.hood);
        this.wristCommands = new WristCommands(this.wrist);
        this.rollerCommands = new RollerCommands(this.rollers);

        this.atShotVel = () -> shooterCommands.isAtVelocity(this.shotSpeed, 1.0);
        this.atHoodAngle = () -> hoodCommands.angleReached(this.hoodAngle, 2.0);
        this.readyToShoot = new Trigger(this.isRotAligned).debounce(0.1);

        this.currentState = SuperstructureState.NONE;
    }

    // FOR LEDS
    public enum SuperstructureState {
        NONE,
        INTAKING,
        PREPPING,
        SHOOTING;

        // basically everytime you want to change the leds, it only sets the state if it has higher
        // priority
        // this is to avoid trying to set the state to two different things repeatedly
        public final int priority;

        SuperstructureState() {
            this.priority = ordinal();
        }
    }

    public SuperstructureState getState() {
        return currentState;
    }

    /**
     * Use this with deadlineFor() to set the state of the superstructure for the duration of a
     * command, it will only change the state if the new state has higher or equal priority than the
     * current state. After the command ends, it will reset the state to NONE.
     */
    public Command requestSetState(SuperstructureState newState) {
        return Commands.run(
                        () -> {
                            if (newState.priority >= currentState.priority) {
                                currentState = newState;
                            }
                        })
                .finallyDo(
                        () -> {
                            currentState = SuperstructureState.NONE;
                        });
    }

    public Command setStateNoneOverride() {
        return Commands.run(() -> currentState = SuperstructureState.NONE);
    }

    public Angle getIntakeAngle() {
        return wantedIntakeAngle;
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
                                        "Shooting/ShotSpeed/leftshootervelocity",
                                        shooterOne.getVelocity())),
                Commands.run(
                        () ->
                                Logger.recordOutput(
                                        "Shooting/ShotSpeed/rightshootervelocity",
                                        shooterTwo.getVelocity())),
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
        return Commands.parallel(logEverything(), prep(), retractIntake(), indexWhenReady());
    }

    public Command bumpShot() {
        return Commands.parallel(
                logEverything(),
                prepForBump(),
                hoodCommands.setAngle(Degrees.of(72)),
                retractIntake(),
                index());
    }

    public Command prep() {
        return shooterCommands
                .setVelocity(shotSpeed)
                .deadlineFor(requestSetState(SuperstructureState.PREPPING));
    }

    public Command prepForBump() {
        return shooterCommands
                .setVelocity(() -> 13.5)
                .deadlineFor(requestSetState(SuperstructureState.PREPPING));
    }

    public Command indexWhenReady() {
        return Commands.sequence(
                Commands.waitUntil(readyToShoot.and(atShotVel).and(atHoodAngle)),
                index().onlyWhile(readyToShoot)
                        .repeatedly()
                        .deadlineFor(requestSetState(SuperstructureState.SHOOTING)));
    }

    public Command index() {
        return Commands.parallel(
                indexerCommands.index(), kickerCommands.kick(), rollerCommands.intake());
    }

    public Command reverse() {
        return Commands.parallel(
                indexerCommands.reverse(), kickerCommands.reverse(), rollerCommands.spit());
    }

    public Command intake() {
        return Commands.parallel(
                        wristCommands.deploy(),
                        // rollerCommands.intakeSpeedScale(robotTracker::getIntakeDirectionSpeed),
                        rollerCommands.intake())
                .deadlineFor(requestSetState(SuperstructureState.INTAKING));
    }

    public Command retractIntake() {
        return Commands.parallel(wristCommands.retract());
    }
}
