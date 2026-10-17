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

    public final RobotTracker robotTracker;

    public final Trigger atShotVel;
    public final BooleanSupplier atHoodAngle;
    public final Trigger readyToShoot;

    public boolean fuelLoaded = false;
    public final Trigger kickerCurrent;
    public final Trigger notLoaded;

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

        this.atShotVel = new Trigger(() -> shooterCommands.isAtVelocity(this.shotSpeed));
        this.atHoodAngle = () -> hoodCommands.angleReached(this.hoodAngle, 2.0);
        this.readyToShoot = new Trigger(this.isRotAligned).debounce(0.1);

        this.kickerCurrent = new Trigger(() -> kicker.getTotalCurrent() > 0.7).debounce(0.1);
        this.notLoaded = new Trigger(() -> !fuelLoaded && RobotState.isEnabled());
    }

    public Command setFuelLoaded(boolean loaded) {
        return Commands.runOnce(() -> fuelLoaded = loaded);
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

    public Command shoot() {
        return Commands.parallel(prep(), indexWhenReady());
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
                                .and(() -> hoodCommands.angleReached(fixedHoodAngle, 2.0))),
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
                Commands.waitUntil(readyToShoot.and(atShotVel).and(atHoodAngle)), index());
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
                wristCommands.deploy(), rollerCommands.intake(), indexerCommands.openLoop(0.3));
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
