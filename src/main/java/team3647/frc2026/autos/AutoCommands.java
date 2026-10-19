package team3647.frc2026.autos;

import choreo.trajectory.SwerveSample;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import team3647.frc2026.constants.FieldConstants;
import team3647.frc2026.constants.ShooterConstants;
import team3647.frc2026.constants.WristConstants;
import team3647.frc2026.subsystems.Superstructure;
import team3647.frc2026.subsystems.Swerve.SwerveDrive;
import team3647.frc2026.util.AllianceFlip;
import team3647.frc2026.util.AutoDrive;

public class AutoCommands extends AutoPathFollowers {
    // pathplanner autos
    //     public Command leftMidRushPathPlanner() {
    //         return followPathPlannerAuto("Left Mid Sweep");
    //     }

    //     public Command rightMidRushPathPlanner() {
    //         return followPathPlannerAuto("Right Mid Sweep");
    //     }


    double spinUpSpeed = 9.0;

    public Command shoot(double time) {
        return Commands.deadline(
                Commands.waitSeconds(time),
                autoAlign(),
                superstructure.shoot(),
                Commands.waitSeconds(2).andThen(superstructure.wristCommands.retract()));
    }

    public double getShootSpeedForEndOfPath(String pathName, boolean mirrored) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            if (mirrored) {
                path = path.mirrorPath();
            }
            List<Pose2d> poses = path.getPathPoses();
            Pose2d endPose = poses.get(poses.size() - 1);
            Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
            Pose2d allianceEndPose = AllianceFlip.flip(endPose, alliance);
            Pose2d hubPose = AllianceFlip.flip(FieldConstants.blueHubPose, alliance);
            double distance =
                    allianceEndPose.getTranslation().minus(hubPose.getTranslation()).getNorm();
            return ShooterConstants.shootSpeedMap.get(distance);
        } catch (Exception e) {
            return spinUpSpeed;
        }
    }

    public Command sweepAndShoot(String pathname, double shootTime, boolean mirror) {
        Command pathFollow =
                mirror ? followMirroredPathPlannerPath(pathname) : followPathPlannerPath(pathname);
        double spinUp = spinUpSpeed;
        return Commands.sequence(
                Commands.deadline(
                        pathFollow,
                        superstructure.intake(),
                        superstructure.shooterCommands.setVelocity(() -> spinUp),
                        superstructure.hoodCommands.setAngle(WristConstants.kMinAngle)),
                shoot(shootTime));
    }

    public Command sweepAndShoot(String pathname, double shootTime) {
        return sweepAndShoot(pathname, shootTime, false);
    }

    public Command sweepShootMirror(String pathname, double shootTime) {
        return sweepAndShoot(pathname, shootTime, true);
    }

    public Command rightMidBumpPP() {
        return Commands.sequence(
                sweepAndShoot("Right Mid Bump 1", 4), sweepAndShoot("Right Mid Bump 2", 10));
    }

    public Command leftMidBumpPP() {
        return Commands.sequence(
                sweepShootMirror("Right Mid Bump 1", 4), sweepShootMirror("Right Mid Bump 2", 10));
    }

    public Command rightBumpLong() {
        return Commands.sequence(
                sweepAndShoot("Right Mid Bump 1", 4), sweepAndShoot("Right Mid Bump 2 Long", 10));
    }

    public Command leftBumpLong() {
        return Commands.sequence(
                sweepShootMirror("Right Mid Bump 1", 4),
                sweepShootMirror("Right Mid Bump 2 Long", 10));
    }

    public Command leftMidRushPathPlanner() {
        return Commands.sequence(
                Commands.deadline(
                        followMirroredPathPlannerPath("Right Mid Sweep 1"),
                        superstructure.intakeWithDelay(),
                        superstructure.shooterCommands.setVelocity(
                                () -> ShooterConstants.kIdleSpeedMid)),
                Commands.sequence(
                        Commands.deadline(
                                followMirroredPathPlannerPath("Right Mid Sweep 2"),
                                superstructure.shooterCommands.setVelocity(() -> spinUpSpeed)),
                        Commands.deadline(
                                Commands.waitSeconds(2.5), autoAlign(), superstructure.shoot())),
                Commands.deadline(
                        followMirroredPathPlannerPath("Right Mid Sweep 3"),
                        superstructure.intakeWithDelay(),
                        superstructure.shooterCommands.setVelocity(
                                () -> ShooterConstants.kIdleSpeedMid)),
                Commands.sequence(
                        Commands.deadline(
                                followMirroredPathPlannerPath("Right Mid Sweep 2"),
                                superstructure.shooterCommands.setVelocity(() -> spinUpSpeed)),
                        Commands.deadline(
                                Commands.waitSeconds(5), autoAlign(), superstructure.shoot())));
    }

    public Command rightMidRushPathPlanner() {
        return Commands.sequence(
                Commands.deadline(
                        followPathPlannerPath("Right Mid Sweep 1"),
                        superstructure.intakeWithDelay(),
                        superstructure.shooterCommands.setVelocity(
                                () -> ShooterConstants.kIdleSpeedMid)),
                Commands.sequence(
                        Commands.deadline(
                                followPathPlannerPath("Right Mid Sweep 2"),
                                superstructure.shooterCommands.setVelocity(() -> spinUpSpeed)),
                        Commands.deadline(
                                Commands.waitSeconds(2.5), autoAlign(), superstructure.shoot())),
                Commands.deadline(
                        followPathPlannerPath("Right Mid Sweep 3"),
                        superstructure.intakeWithDelay(),
                        superstructure.shooterCommands.setVelocity(
                                () -> ShooterConstants.kIdleSpeedMid)),
                Commands.sequence(
                        Commands.deadline(
                                followPathPlannerPath("Right Mid Sweep 2"),
                                superstructure.shooterCommands.setVelocity(() -> spinUpSpeed)),
                        Commands.deadline(
                                Commands.waitSeconds(5), autoAlign(), superstructure.shoot())));
    }

    public Command depot() {
        return Commands.sequence(
                Commands.deadline(followPathPlannerPath("Depot 1"), superstructure.intake()),
                followPathPlannerPath("Depot 2"),
                Commands.parallel(autoAlign(), superstructure.shoot()));
    }

    // choreo autos, not in-use

    // preload only
    public Command preloadAuto() {
        return Commands.parallel(superstructure.shoot(), autoAlign());
    }

    // mid rushes
    private final String rightMidRush = "RightMidRush";
    private final String leftMidRush = "LeftMidRush";

    public Command midRushAuto(String pathname) {
        return Commands.sequence(
                // superstructure.bumpKicker().withTimeout(0.35),
                midRushSegmentOne(pathname),
                midRushSegmentTwo(pathname).withTimeout(4),
                Commands.parallel(superstructure.shoot(), autoAlign()));
    }

    public Command midRushSegmentOne(String pathname) {
        return Commands.deadline(
                followChoreoPath(getTrajSplit(pathname, 0)),
                Commands.sequence(Commands.waitSeconds(0.8), superstructure.intake()));
    }

    public Command midRushSegmentTwo(String pathname) {
        return Commands.deadline(
                followChoreoPath(getTrajSplit(pathname, 1))
                // superstructure.retractIntake(),
                );
    }

    // double mid rushes
    public Command doubleMidRushAuto(String pathname) {
        return Commands.sequence(
                midRushAuto(pathname).withTimeout(9.5),
                superstructure.retractIntake().withTimeout(0.2),
                midRushAuto(pathname + "Close"));
    }

    // double mid rush loops
    public Command doubleMidRushLoopAuto(String pathname) {
        return Commands.sequence(
                midRushAuto(pathname).withTimeout(9.5),
                superstructure.retractIntake().withTimeout(0.2),
                midRushAuto(pathname + "CloseLoop"));
    }

    public List<AutoMode> autosList;

    public final AutoMode leftMidRushPP;
    public final AutoMode rightMidRushPP;
    public final AutoMode rightMidBump;
    public final AutoMode leftMidBump;
    public final AutoMode rightBumpLong;
    public final AutoMode leftBumpLong;
    public final AutoMode depot;
    public final AutoMode empty;
    public final AutoMode preload;

    public AutoCommands(
            SwerveDrive swerve,
            Superstructure superstructure,
            Supplier<Twist2d> autoDriveVelocities,
            BooleanSupplier hasTarget,
            AutoDrive autoDrive) {
        super(swerve, superstructure, autoDriveVelocities, hasTarget, autoDrive);

        this.leftMidRushPP =
                new AutoMode(
                        leftMidRushPathPlanner(),
                        getInitialPPMirroredLazy("Right Mid Sweep 1"),
                        "Left Mid Sweep");

        this.rightMidRushPP =
                new AutoMode(
                        rightMidRushPathPlanner(),
                        getInitialPPLazy("Right Mid Sweep 1"),
                        "Right Mid Sweep");

        this.rightMidBump =
                new AutoMode(
                        rightMidBumpPP(), getInitialPPLazy("Right Mid Bump 1"), "Right Mid Bump");

        this.leftMidBump =
                new AutoMode(
                        leftMidBumpPP(),
                        getInitialPPMirroredLazy("Right Mid Bump 1"),
                        "Left Mid Bump");

        this.rightBumpLong =
                new AutoMode(
                        rightBumpLong(), getInitialPPLazy("Right Mid Bump 1"), "Right Bump Long");

        this.leftBumpLong =
                new AutoMode(
                        leftBumpLong(),
                        getInitialPPMirroredLazy("Right Mid Bump 1"),
                        "Left Bump Long");

        this.depot = new AutoMode(depot(), getInitialPPLazy("Depot 1"), "Depot");

        this.preload = new AutoMode(preloadAuto(), new Pose2d(), "preload");

        this.empty = new AutoMode(Commands.none(), new Pose2d(), "empty");

        autosList =
                List.of(
                        empty,
                        leftMidRushPP,
                        rightMidRushPP,
                        rightMidBump,
                        leftMidBump,
                        rightBumpLong,
                        leftBumpLong,
                        preload,
                        depot);
    }

    /**
     * Resolve + cache every auto's starting pose and generate one trajectory up front, so the
     * PathPlanner file parsing and first-run JIT/class-loading happen at boot instead of
     * synchronously in {@code autonomousInit}, where they would stall the first path at auto start.
     * Safe to call while disabled; it never commands the drivetrain. Call once from robot init.
     */
    public void warmup(RobotConfig config) {
        // Touch each auto's start pose so the .path file read + caching happens now, not at enable.
        for (AutoMode mode : autosList) {
            try {
                mode.getStartingPose();
            } catch (Exception e) {
                // best-effort warmup; ignore
            }
        }

        // Generate one trajectory so PathPlanner's generation code is loaded/JIT'd before the
        // first path's initialize() runs at auto start.
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile("Right Mid Bump 1");
            Rotation2d startRot =
                    path.getStartingHolonomicPose()
                            .map(Pose2d::getRotation)
                            .orElse(new Rotation2d());
            path.generateTrajectory(new ChassisSpeeds(), startRot, config);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public BooleanSupplier hasTarget() {
        return hasTarget;
    }

    public interface ChoreoController extends BiFunction<Pose2d, SwerveSample, ChassisSpeeds> {}
}
