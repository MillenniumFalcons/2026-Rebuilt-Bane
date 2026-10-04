package team3647.frc2026.autos;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import team3647.frc2026.constants.ShooterConstants;
import team3647.frc2026.constants.WristConstants;
import team3647.frc2026.subsystems.Superstructure;
import team3647.frc2026.subsystems.Swerve.SwerveDrive;
import team3647.frc2026.util.AutoDrive;

public class AutoCommands extends AutoPathFollowers {
    // pathplanner autos
    //     public Command leftMidRushPathPlanner() {
    //         return followPathPlannerAuto("Left Mid Sweep");
    //     }

    //     public Command rightMidRushPathPlanner() {
    //         return followPathPlannerAuto("Right Mid Sweep");
    //     }

    double spinUpSpeed = 10.0;

    public Command shoot(double time) {
        return Commands.deadline(
                Commands.waitSeconds(time), autoAlign(), superstructure.shoot());
    }

    public Command sweepAndShoot(String pathname, double shootTime) {
        return Commands.sequence(
                Commands.deadline(
                        followPathPlannerPath(pathname),
                        superstructure.intakeWithDelay(),
                        superstructure.hoodCommands.setAngle(WristConstants.kMinAngle)),
                shoot(shootTime)
        );
    }

    public Command sweepShootMirror(String pathname, double shootTime) {
        return Commands.sequence(
                Commands.deadline(
                        followMirroredPathPlannerPath(pathname),
                        superstructure.intakeWithDelay(),
                        superstructure.hoodCommands.setAngle(WristConstants.kMinAngle)),
                shoot(shootTime)
        );
    }
    public Command rightMidBumpPP() {
        return Commands.sequence(
                sweepAndShoot("Right Mid Bump 1", 4),
                sweepAndShoot("Right Mid Bump 2", 10)
        );
    }

    public Command leftMidBumpPP() {
        return Commands.sequence(
                sweepShootMirror("Right Mid Bump 1", 2.5),
                sweepShootMirror("Right Mid Bump 2", 10)
        );
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

    public List<AutoMode> redAutosList;
    public List<AutoMode> blueAutosList;

    public final AutoMode leftMidRushBluePP;
    public final AutoMode leftMidRushRedPP;

    public final AutoMode rightMidRushBluePP;
    public final AutoMode rightMidRushRedPP;

    public final AutoMode rightMidBump;
    public final AutoMode leftMidBump;

    public final AutoMode depotBlue;
    public final AutoMode depotRed;

    public final AutoMode blueEmpty;
    public final AutoMode preloadBlue;

    public final AutoMode redEmpty;
    public final AutoMode preloadRed;

    public AutoCommands(
            SwerveDrive swerve,
            Superstructure superstructure,
            Supplier<Twist2d> autoDriveVelocities,
            BooleanSupplier hasTarget,
            AutoDrive autoDrive) {
        super(swerve, superstructure, autoDriveVelocities, hasTarget, autoDrive);

        this.leftMidRushBluePP =
                new AutoMode(
                        leftMidRushPathPlanner(),
                        getInitialPPMirrored("Right Mid Sweep 1", Alliance.Blue),
                        "Left Mid Sweep");
        this.leftMidRushRedPP =
                new AutoMode(
                        leftMidRushPathPlanner(),
                        getInitialPPMirrored("Right Mid Sweep 1", Alliance.Red),
                        "Left Mid Sweep");

        this.rightMidRushBluePP =
                new AutoMode(
                        rightMidRushPathPlanner(),
                        getInitialPP("Right Mid Sweep", Alliance.Blue),
                        "Right Mid Sweep");
        this.rightMidRushRedPP =
                new AutoMode(
                        rightMidRushPathPlanner(),
                        getInitialPP("Right Mid Sweep", Alliance.Red),
                        "Right Mid Sweep");

        this.rightMidBump = 
                new AutoMode(
                        rightMidBumpPP(), 
                        getInitialPP("Right Mid Bump 1", Alliance.Red),
                        "Right Mid Bump");

        this.leftMidBump = 
                new AutoMode(
                        leftMidBumpPP(),
                        getInitialPPMirrored(("Right Mid Bump 1"), Alliance.Red), 
                        "Left Mid Bump");

        this.depotBlue = new AutoMode(depot(), getInitialPP("Depot", Alliance.Blue), "Depot");
        this.depotRed = new AutoMode(depot(), getInitialPP("Depot", Alliance.Red), "Depot");

        this.preloadBlue = new AutoMode(preloadAuto(), new Pose2d(), "preload");
        this.preloadRed = new AutoMode(preloadAuto(), new Pose2d(), "preload");

        this.blueEmpty =
                new AutoMode(
                        Commands.none(), Pose2d.kZero.rotateBy(Rotation2d.k180deg), "blue empty");

        this.redEmpty = new AutoMode(Commands.none(), Pose2d.kZero, "red empty");

        blueAutosList =
                List.of(blueEmpty, leftMidRushBluePP, rightMidRushBluePP, preloadBlue, depotBlue);
        redAutosList = List.of(redEmpty, leftMidRushRedPP, rightMidRushRedPP, rightMidBump, leftMidBump, preloadRed, depotRed);
    }

    public BooleanSupplier hasTarget() {
        return hasTarget;
    }

    public interface ChoreoController extends BiFunction<Pose2d, SwerveSample, ChassisSpeeds> {}
}
