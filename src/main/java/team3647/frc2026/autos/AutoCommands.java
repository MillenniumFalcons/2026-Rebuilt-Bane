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

    public Command leftMidRushPathPlanner() {
        return Commands.sequence(
                Commands.deadline(
                        followPathPlannerPath("Left Mid Sweep 1"),
                        superstructure.intakeWithDelay()),
                Commands.sequence(
                        Commands.parallel(
                                followPathPlannerPath("Left Mid Sweep 2"), superstructure.prep()),
                        Commands.deadline(
                                Commands.waitSeconds(2.5), autoAlign(), superstructure.shoot())),
                Commands.deadline(
                        followPathPlannerPath("Left Mid Sweep 3"),
                        superstructure.intakeWithDelay()),
                Commands.sequence(
                        Commands.parallel(
                                followPathPlannerPath("Left Mid Sweep 2"), superstructure.prep()),
                        Commands.deadline(
                                Commands.waitSeconds(2.5), autoAlign(), superstructure.shoot())),
                Commands.deadline(
                        followPathPlannerPath("Left Mid Sweep 4"),
                        superstructure.intakeWithDelay()));
    }

    public Command rightMidRushPathPlanner() {
        return Commands.sequence(
                Commands.deadline(
                        followPathPlannerPath("Right Mid Sweep 1"),
                        superstructure.intakeWithDelay()),
                Commands.sequence(
                        Commands.parallel(
                                followPathPlannerPath("Right Mid Sweep 2"), superstructure.prep()),
                        Commands.deadline(
                                Commands.waitSeconds(2.5), autoAlign(), superstructure.shoot())),
                Commands.deadline(
                        followPathPlannerPath("Right Mid Sweep 3"),
                        superstructure.intakeWithDelay()),
                Commands.sequence(
                        Commands.parallel(
                                followPathPlannerPath("Right Mid Sweep 2"), superstructure.prep()),
                        Commands.deadline(
                                Commands.waitSeconds(2.5), autoAlign(), superstructure.shoot())),
                Commands.deadline(
                        followPathPlannerPath("Right Mid Sweep 4"),
                        superstructure.intakeWithDelay()));
    }

    // actual autos

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

    public final AutoMode blueEmpty;
    public final AutoMode preloadBlue;
    public final AutoMode rightDoubleMidRushBlue;
    public final AutoMode leftDoubleMidRushBlue;
    public final AutoMode rightDoubleMidRushLoopBlue;
    public final AutoMode leftDoubleMidRushLoopBlue;

    public final AutoMode redEmpty;
    public final AutoMode preloadRed;
    public final AutoMode rightDoubleMidRushRed;
    public final AutoMode leftDoubleMidRushRed;
    public final AutoMode rightDoubleMidRushLoopRed;
    public final AutoMode leftDoubleMidRushLoopRed;

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
                        getInitialPP("Left Mid Sweep", Alliance.Blue),
                        "Left Mid Sweep");
        this.leftMidRushRedPP =
                new AutoMode(
                        leftMidRushPathPlanner(),
                        getInitialPP("Left Mid Sweep", Alliance.Red),
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

        this.preloadBlue = new AutoMode(preloadAuto(), new Pose2d(), "preload");
        this.preloadRed = new AutoMode(preloadAuto(), new Pose2d(), "preload");

        this.blueEmpty =
                new AutoMode(
                        Commands.none(), Pose2d.kZero.rotateBy(Rotation2d.k180deg), "blue empty");
        this.rightDoubleMidRushBlue =
                new AutoMode(
                        doubleMidRushAuto(rightMidRush),
                        getInitial(getTraj(rightMidRush), Alliance.Blue),
                        "double" + rightMidRush);
        this.leftDoubleMidRushBlue =
                new AutoMode(
                        doubleMidRushAuto(leftMidRush),
                        getInitial(getTraj(leftMidRush), Alliance.Blue),
                        "double" + leftMidRush);
        this.rightDoubleMidRushLoopBlue =
                new AutoMode(
                        doubleMidRushLoopAuto(rightMidRush),
                        getInitial(getTraj(rightMidRush), Alliance.Blue),
                        "doubleLoop" + rightMidRush);
        this.leftDoubleMidRushLoopBlue =
                new AutoMode(
                        doubleMidRushLoopAuto(leftMidRush),
                        getInitial(getTraj(leftMidRush), Alliance.Blue),
                        "doubleLoop" + leftMidRush);

        this.redEmpty = new AutoMode(Commands.none(), Pose2d.kZero, "red empty");
        this.rightDoubleMidRushRed =
                new AutoMode(
                        doubleMidRushAuto(rightMidRush),
                        getInitial(getTraj(rightMidRush), Alliance.Red),
                        "double" + rightMidRush);
        this.leftDoubleMidRushRed =
                new AutoMode(
                        doubleMidRushAuto(leftMidRush),
                        getInitial(getTraj(leftMidRush), Alliance.Red),
                        "double" + leftMidRush);
        this.rightDoubleMidRushLoopRed =
                new AutoMode(
                        doubleMidRushLoopAuto(rightMidRush),
                        getInitial(getTraj(rightMidRush), Alliance.Red),
                        "doubleLoop" + rightMidRush);
        this.leftDoubleMidRushLoopRed =
                new AutoMode(
                        doubleMidRushLoopAuto(leftMidRush),
                        getInitial(getTraj(leftMidRush), Alliance.Red),
                        "doubleLoop" + leftMidRush);

        blueAutosList =
                List.of(
                        blueEmpty,
                        leftMidRushBluePP,
                        rightMidRushBluePP,
                        rightDoubleMidRushBlue,
                        leftDoubleMidRushBlue,
                        rightDoubleMidRushLoopBlue,
                        leftDoubleMidRushLoopBlue,
                        preloadBlue);
        redAutosList =
                List.of(
                        redEmpty,
                        leftMidRushRedPP,
                        rightMidRushRedPP,
                        rightDoubleMidRushRed,
                        leftDoubleMidRushRed,
                        rightDoubleMidRushLoopRed,
                        leftDoubleMidRushLoopRed,
                        preloadRed);
    }

    public BooleanSupplier hasTarget() {
        return hasTarget;
    }

    public interface ChoreoController extends BiFunction<Pose2d, SwerveSample, ChassisSpeeds> {}
}
