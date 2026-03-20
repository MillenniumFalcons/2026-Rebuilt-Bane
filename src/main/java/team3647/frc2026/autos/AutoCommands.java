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
                midRushSegmentTwo(pathname),
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
                midRushAuto(pathname).withTimeout(11.0), midRushAuto(pathname + "Close"));
    }

    public List<AutoMode> redAutosList;
    public List<AutoMode> blueAutosList;

    public final AutoMode blueEmpty;
    public final AutoMode preloadBlue;
    public final AutoMode rightMidRushBlue;
    public final AutoMode leftMidRushBlue;
    public final AutoMode rightDoubleMidRushBlue;
    public final AutoMode leftDoubleMidRushBlue;

    public final AutoMode redEmpty;
    public final AutoMode preloadRed;
    public final AutoMode rightMidRushRed;
    public final AutoMode leftMidRushRed;
    public final AutoMode rightDoubleMidRushRed;
    public final AutoMode leftDoubleMidRushRed;

    public AutoCommands(
            SwerveDrive swerve,
            Superstructure superstructure,
            Supplier<Twist2d> autoDriveVelocities,
            BooleanSupplier hasTarget,
            AutoDrive autoDrive) {
        super(swerve, superstructure, autoDriveVelocities, hasTarget, autoDrive);

        this.preloadBlue = new AutoMode(preloadAuto(), new Pose2d(), "preload");
        this.preloadRed = new AutoMode(preloadAuto(), new Pose2d(), "preload");

        this.blueEmpty =
                new AutoMode(
                        Commands.none(), Pose2d.kZero.rotateBy(Rotation2d.k180deg), "blue empty");
        this.rightMidRushBlue =
                new AutoMode(
                        midRushAuto(rightMidRush),
                        getInitial(getTraj(rightMidRush), Alliance.Blue),
                        rightMidRush);
        this.leftMidRushBlue =
                new AutoMode(
                        midRushAuto(leftMidRush),
                        getInitial(getTraj(leftMidRush), Alliance.Blue),
                        leftMidRush);
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

        this.redEmpty = new AutoMode(Commands.none(), Pose2d.kZero, "red empty");
        this.rightMidRushRed =
                new AutoMode(
                        midRushAuto(rightMidRush),
                        getInitial(getTraj(rightMidRush), Alliance.Red),
                        rightMidRush);
        this.leftMidRushRed =
                new AutoMode(
                        midRushAuto(leftMidRush),
                        getInitial(getTraj(leftMidRush), Alliance.Red),
                        leftMidRush);
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

        blueAutosList =
                List.of(
                        blueEmpty,
                        rightDoubleMidRushBlue,
                        leftDoubleMidRushBlue,
                        preloadBlue,
                        rightMidRushBlue,
                        leftMidRushBlue);
        redAutosList =
                List.of(
                        redEmpty,
                        rightDoubleMidRushRed,
                        leftDoubleMidRushRed,
                        preloadRed,
                        rightMidRushRed,
                        leftMidRushRed);
    }

    public BooleanSupplier hasTarget() {
        return hasTarget;
    }

    public interface ChoreoController extends BiFunction<Pose2d, SwerveSample, ChassisSpeeds> {}
}
