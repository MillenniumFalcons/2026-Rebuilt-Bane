package team3647.frc2026.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotState;
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
import team3647.frc2026.constants.HoodConstants;
import team3647.frc2026.constants.ShooterConstants;
import team3647.frc2026.util.RobotTracker;
import team3647.lib.team6328.VirtualSubsystem;

public class Superstructure extends VirtualSubsystem {

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

    // Fixed shooting distance (meters). Hood angle and shot speed are looked up from the
    // interpolation tables at this distance; no auto-align is used (driver aims manually).
    public static final double kFixedShootDistance = 3.35;

    private final DoubleSupplier fixedShotSpeed =
            () -> ShooterConstants.shootSpeedMap.get(kFixedShootDistance);
    private final DoubleSupplier fixedHoodAngle =
            () -> HoodConstants.hoodAngleMap.get(kFixedShootDistance);

    public Angle wantedIntakeAngle = Degrees.of(70);
    public final BooleanSupplier isRotAligned;

    // True while an auto-align is actively driving rotation (i.e. the robot is "trying to align").
    public final BooleanSupplier isAligning;

    // True while the robot is in feed mode (past the trench). Feeding uses a looser 2 rps
    // shot-velocity
    // tolerance so indexing starts sooner; shooting keeps the stricter isAtVelocity tolerance.
    public final BooleanSupplier isFeeding;

    // Shot-velocity tolerance (rps) used to gate indexing while feeding.
    public static final double kFeedVelocityTolerance = 2.0;

    public final RobotTracker robotTracker;

    public final Trigger atShotVel;
    public final BooleanSupplier atHoodAngle;
    public final Trigger readyToShoot;

    // LED state: actively firing — rotation aligned, shooter at speed, and hood at angle.
    public final Trigger shooting;
    // LED state: spinning up the shooter / aligning rotation, but not yet ready to fire.
    public final Trigger prepping;

    public boolean fuelLoaded = false;
    public final Trigger kickerCurrent;
    public final Trigger notLoaded;

    // Manual override: when true, the indexWhenReady* gates fire immediately instead of waiting
    // for alignment/velocity/hood. Lets the co-driver force-start feeding on an already-running
    // shoot() without scheduling a competing command that would interrupt the spin-up.
    public boolean forceIndex = false;

    public Superstructure(
            Shooter shooter,
            Indexer indexer,
            Hood hood,
            Kicker kicker,
            Wrist wrist,
            Rollers rollers,
            RobotTracker robotTracker,
            DoubleSupplier hoodAngle,
            DoubleSupplier shotSpeed,
            BooleanSupplier isRotAligned,
            BooleanSupplier isFeeding,
            BooleanSupplier isAligning) {
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
        this.isFeeding = isFeeding;
        this.isAligning = isAligning;

        this.shooterCommands = new ShooterCommands(this.shooter);
        this.indexerCommands = new IndexerCommands(this.indexer);
        this.kickerCommands = new KickerCommands(this.kicker);
        this.hoodCommands = new HoodCommands(this.hood);
        this.wristCommands = new WristCommands(this.wrist);
        this.rollerCommands = new RollerCommands(this.rollers);
        this.rollerCommands.setVxSupplier(this.robotTracker::getIntakeDirectionSpeed);

        this.atShotVel =
                new Trigger(
                        () ->
                                isFeeding.getAsBoolean()
                                        ? shooterCommands.isAtVelocity(
                                                this.shotSpeed, kFeedVelocityTolerance)
                                        : shooterCommands.isAtVelocity(this.shotSpeed));
        this.atHoodAngle = () -> hoodCommands.angleReached(this.hoodAngle, 2.0);
        this.readyToShoot = new Trigger(this.isRotAligned).debounce(0.1);

        this.shooting = this.readyToShoot.and(this.atShotVel).and(this.atHoodAngle);

        // Trying to spin up the shooter: a shoot/prep command is driving the shooter instead of
        // the idle default command.
        Trigger spinningUp =
                new Trigger(
                        () ->
                                shooter.getCurrentCommand() != null
                                        && shooter.getCurrentCommand()
                                                != shooter.getDefaultCommand());
        // Trying to align rotation.
        Trigger aligning = new Trigger(this.isAligning);
        // Prepping = trying to spin up but not yet at shot speed, OR trying to align but not yet
        // rotationally aligned.
        this.prepping =
                spinningUp
                        .and(this.atShotVel.negate())
                        .or(aligning.and(() -> !this.isRotAligned.getAsBoolean()));

        this.kickerCurrent = new Trigger(() -> kicker.getTotalCurrent() > 0.7).debounce(0.1);
        this.notLoaded = new Trigger(() -> !fuelLoaded && RobotState.isEnabled());
    }

    public Command setFuelLoaded(boolean loaded) {
        return Commands.runOnce(() -> fuelLoaded = loaded);
    }

    /**
     * Hold to force any active shooting mode to start indexing immediately, bypassing the
     * alignment/velocity/hood gate. Requires no subsystems, so it won't interrupt the running
     * shoot/prep command.
     */
    public Command forceIndex() {
        return Commands.startEnd(() -> forceIndex = true, () -> forceIndex = false);
    }

    public Angle getIntakeAngle() {
        return wantedIntakeAngle;
    }

    public Command feed() {
        return Commands.sequence(
                // Commands.waitUntil(readyToShoot),
                Commands.parallel(deployIntake(), reverse()));
    }

    public void logShooting() {
        double wantedSpeed = Math.round(shotSpeed.getAsDouble() * 100.0) / 100.0;
        Logger.recordOutput("Shooting/ShotSpeed/wantedshotspeed", wantedSpeed);
        Logger.recordOutput(
                "Shooting/ShotSpeed/shootervelocity",
                Math.round(shooter.getVelocity() * 100.0) / 100.0);
        Logger.recordOutput("Shooting/ShotSpeed/atshotvel", atShotVel.getAsBoolean());
        Logger.recordOutput(
                "Shooting/Hood/wantedhoodeangle",
                Math.round(hoodAngle.getAsDouble() * 100.0) / 100.0);
        Logger.recordOutput(
                "Shooting/Hood/currenthoodangle", Math.round(hood.getPosition() * 100.0) / 100.0);
        Logger.recordOutput("Shooting/Hood/athoodangle", atHoodAngle.getAsBoolean());
        Logger.recordOutput("Shooting/Rotation/isrotaligned", isRotAligned.getAsBoolean());
    }

    @Override
    public void periodic() {
        logShooting();
    }

    // Maximum age (seconds) of an accepted vision pose for shootWithVision() to allow firing.
    public static final double kVisionRecencyWindow = 1.0;

    public Command shoot() {
        return Commands.parallel(prep(), indexWhenReady());
    }

    /**
     * Same as {@link #shoot()} but only indexes (fires) while a vision pose has been accepted
     * within the last {@link #kVisionRecencyWindow} seconds. Still spins up and aligns normally; if
     * vision goes stale the shot gate holds until a fresh vision frame comes in.
     */
    public Command shootWithVision() {
        return Commands.parallel(prep(), indexWhenReadyWithVision());
    }

    public Command indexWhenReadyWithVision() {
        return Commands.sequence(
                Commands.waitUntil(
                        readyToShoot
                                .and(atShotVel)
                                .and(atHoodAngle)
                                .and(() -> robotTracker.hasRecentVision(kVisionRecencyWindow))
                                .or(() -> forceIndex)),
                index());
    }

    /**
     * Same as {@link #shoot()} but does not wait for rotation alignment before indexing. Still
     * spins up and waits for shot velocity and hood angle; the driver aims rotation manually.
     */
    public Command shootNoAlign() {
        return Commands.parallel(prep(), indexWhenReadyNoAlign());
    }

    public Command indexWhenReadyNoAlign() {
        return Commands.sequence(
                Commands.waitUntil(atShotVel.and(atHoodAngle).or(() -> forceIndex)), index());
    }

    /**
     * Same as {@link #shoot()} but with hood angle and shot speed hardcoded from the {@link
     * #kFixedShootDistance} lookup, with no auto-align (driver aims manually).
     */
    public Command shootFixed() {
        return Commands.parallel(prepFixed(), indexWhenReadyFixed());
    }

    public Command prepFixed() {
        return shooterCommands
                .setVelocity(fixedShotSpeed)
                .alongWith(hoodCommands.setAngleDeg(fixedHoodAngle::getAsDouble));
    }

    public Command indexWhenReadyFixed() {
        return Commands.sequence(
                Commands.waitUntil(
                        readyToShoot
                                .and(() -> shooterCommands.isAtVelocity(fixedShotSpeed))
                                .and(() -> hoodCommands.angleReached(fixedHoodAngle, 2.0))
                                .or(() -> forceIndex)),
                indexFixed());
    }

    public Command indexFixed() {
        return Commands.parallel(
                indexerCommands.index(),
                kickerCommands.matchShooterSpeed(fixedShotSpeed),
                setFuelLoaded(false));
    }

    public Command shootInstantly() {
        return Commands.parallel(
                prepKids(),
                Commands.sequence(
                        Commands.waitUntil(
                                () ->
                                        shooterCommands.isAtVelocity(
                                                () -> ShooterConstants.kidSpeed)),
                        index()));
    }

    public Command prepShoot() {
        return shooterCommands.setVelocity(shotSpeed);
    }

    public Command prep() {
        return shooterCommands
                .setVelocity(shotSpeed)
                .alongWith(hoodCommands.setAngleDeg(hoodAngle::getAsDouble));
    }

    public Command prepKids() {
        return shooterCommands.setVelocity(() -> ShooterConstants.kidSpeed);
    }

    public Command bumpShot() {
        return Commands.parallel(prepForBump(), index());
    }

    public Command prepForBump() {
        return shooterCommands.setVelocity(() -> 8.5);
    }

    public Command indexWhenReady() {
        return Commands.sequence(
                Commands.waitUntil(
                        readyToShoot.and(atShotVel).and(atHoodAngle).or(() -> forceIndex)),
                index());
        // .onlyWhile(readyToShoot)
        //         .repeatedly());
    }

    public Command index() {
        return Commands.parallel(
                indexerCommands.index(),
                kickerCommands.matchShooterSpeed(shotSpeed),
                setFuelLoaded(false));
    }

    public Command reverse() {
        return Commands.parallel(
                indexerCommands.reverse(), kickerCommands.reverse(), rollerCommands.spit());
    }

    public Command intake() {
        return Commands.parallel(
                wristCommands.deploy(), rollerCommands.intake(), indexerCommands.openLoop(-0.3));
    }

    public Command intakeNoIndex() {
        return Commands.parallel(wristCommands.deploy(), rollerCommands.intake());
    }

    public Command intakeFast() {
        return Commands.parallel(
                wristCommands.deploy(), rollerCommands.intakeFast(), indexerCommands.openLoop(0.3));
    }

    public Command intakeWithDelay() {
        return Commands.sequence(
                Commands.waitSeconds(0.1),
                Commands.parallel(wristCommands.deploy(), rollerCommands.intake()));
    }

    public Command retractIntake() {
        return Commands.parallel(wristCommands.retract(), indexerCommands.index());
    }

    public Command deployIntake() {
        return Commands.parallel(wristCommands.deploy());
    }

    public Command intakeDown() {
        return Commands.parallel(wristCommands.deploy(), rollerCommands.intake());
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
                Commands.sequence(Commands.waitSeconds(3.0), index()));
    }

    public Command loadWhenDetected() {
        return Commands.sequence(
                Commands.race(kickerCommands.preload(), Commands.waitUntil(kickerCurrent)),
                Commands.waitSeconds(2.0),
                kickerCommands.spinRotations(0.5));
    }
}
