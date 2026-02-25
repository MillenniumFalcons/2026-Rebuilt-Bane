package team3647.frc2026.autos;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import team3647.frc2026.constants.AutoConstants;
import team3647.frc2026.subsystems.Superstructure;
import team3647.frc2026.subsystems.SwerveDrive;
import team3647.frc2026.util.AutoDrive;
import team3647.frc2026.util.PoseUtils;
import team3647.lib.team9442.AllianceObserver;

public class AutoCommands implements AllianceObserver {

    // private final Rectangle2d BlueLSource =
    //         new Rectangle2d(
    //                 new Translation2d(0, FieldConstants.kFieldLengthM), new Translation2d(4,
    // 5.5));
    // private final Rectangle2d BlueRSource =
    //         new Rectangle2d(Translation2d.kZero, new Translation2d(4, 2.5));
    // private final Rectangle2d RedLSource =
    //         new Rectangle2d(
    //                 AllianceFlip.flip(new Translation2d(0, FieldConstants.kFieldLengthM)),
    //                 AllianceFlip.flip(new Translation2d(4, 5.5)));
    // private final Rectangle2d RedRSource =
    //         new Rectangle2d(
    //                 AllianceFlip.flip(Translation2d.kZero),
    //                 AllianceFlip.flip(new Translation2d(4, 2.5)));

    private final PIDController rotController = new PIDController(1.5, 0, 0);

    // actual autos

    // mid rushes
    private final String rightMidRush = "RightMidRush";
    private final String leftMidRush = "LeftMidRush";

    public Command midRushAuto(String pathname) {
        return Commands.sequence(
                midRushSegmentOne(pathname),
                midRushSegmentTwo(pathname),
                Commands.parallel(superstructure.shoot(), autoDrive.enableAutoDrive()));
    }

    public Command midRushSegmentOne(String pathname) {
        return Commands.parallel(
                followChoreoPath(getTrajSplit(pathname, 0)),
                Commands.sequence(Commands.waitSeconds(0.5), superstructure.intake()));
    }

    public Command midRushSegmentTwo(String pathname) {
        return Commands.parallel(
                followChoreoPath(getTrajSplit(pathname, 1)), superstructure.retractIntake());
    }

    // outpost
    private final String outpost = "Outpost";

    public Command outpostAuto() {
        return Commands.sequence(
                followChoreoPath(getTrajSplit(outpost, 0)),
                Commands.waitSeconds(1.5),
                followChoreoPath(getTrajSplit(outpost, 1)),
                Commands.parallel(superstructure.shoot(), autoAlign()));
    }

    public Command autoAlign() {
        return Commands.run(() -> swerve.drive(0, 0, autoDriveVelocities.get().dtheta), swerve);
    }

    // regular pathfollower
    public Command followChoreoPath(Trajectory<SwerveSample> traj) {
        return customPathFollower(
                traj,
                choreoSwerveController(
                        AutoConstants.autoXController,
                        AutoConstants.autoYController,
                        rotController),
                (speeds) -> {
                    swerve.drive(
                            speeds.vxMetersPerSecond,
                            speeds.vyMetersPerSecond,
                            speeds.omegaRadiansPerSecond);
                },
                swerve::getOdoPose);
    }

    // pathfollower with autoalign overrides
    public Command followChoreoPathWithAutoAlignOverride(Trajectory<SwerveSample> traj) {
        return customPathFollower(
                        traj,
                        choreoSwerveController(
                                AutoConstants.autoXController,
                                AutoConstants.autoYController,
                                AutoConstants.rotController),
                        (ChassisSpeeds speeds) -> {
                            var motionRotComponent = speeds.omegaRadiansPerSecond;
                            var motionXComponent = speeds.vxMetersPerSecond * 0.5;
                            var motionYComponent = speeds.vyMetersPerSecond * 0.5;

                            if (this.hasTarget.getAsBoolean()) {
                                motionXComponent = autoDriveVelocities.get().dx;
                                motionYComponent = autoDriveVelocities.get().dy;
                            }

                            swerve.drive(motionXComponent, motionYComponent, motionRotComponent);
                        },
                        swerve::getOdoPose)
                .andThen(Commands.runOnce(() -> swerve.drive(0, 0, 0), swerve));
    }

    // HELPERS

    public Command customPathFollower(
            Trajectory<SwerveSample> traj,
            ChoreoController controller,
            Consumer<ChassisSpeeds> outputChassisSpeeds,
            Supplier<Pose2d> poseSupplier) {

        Timer timer = new Timer();

        return new FunctionalCommand(
                timer::restart,
                () -> {
                    outputChassisSpeeds.accept(
                            controller.apply(
                                    poseSupplier.get(),
                                    traj.sampleAt(timer.get(), isRed())
                                            .orElse(AutoConstants.kEmptySample)));
                },
                (interrupted) -> {
                    outputChassisSpeeds.accept(new ChassisSpeeds());
                    timer.stop();
                },
                () -> PoseUtils.inCircle(poseSupplier.get(), traj.getFinalPose(isRed()).get(), 0.1),
                swerve);
    }

    public static ChoreoController choreoSwerveController(
            PIDController xController,
            PIDController yController,
            PIDController rotationController) {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        return (pose, referenceState) -> {
            double xFF = referenceState.getChassisSpeeds().vxMetersPerSecond;
            double yFF = referenceState.getChassisSpeeds().vyMetersPerSecond;
            double rotationFF = referenceState.getChassisSpeeds().omegaRadiansPerSecond;

            double xFeedback = xController.calculate(pose.getX(), referenceState.getPose().getX());
            double yFeedback = yController.calculate(pose.getY(), referenceState.getPose().getY());
            double rotationFeedback =
                    rotationController.calculate(
                            pose.getRotation().getRadians(),
                            referenceState.getPose().getRotation().getRadians());

            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    xFF + xFeedback,
                    yFF + yFeedback,
                    rotationFF + rotationFeedback,
                    pose.getRotation());
        };
    }

    @SuppressWarnings("unchecked")
    public Trajectory<SwerveSample> getTraj(String path) {
        var trajMaybe = Choreo.loadTrajectory(path);

        if (trajMaybe.isPresent()) {
            return (Trajectory<SwerveSample>) trajMaybe.get();
        } else {
            return AutoConstants.kEmptyTraj;
        }
    }

    @SuppressWarnings("unchecked")
    public Trajectory<SwerveSample> getTrajSplit(String path, int splitIndex) {
        var trajMaybe = Choreo.loadTrajectory(path);

        if (trajMaybe.isPresent()) {
            var splitMaybe = trajMaybe.get().getSplit(splitIndex);

            if (splitMaybe.isPresent()) {
                return (Trajectory<SwerveSample>) splitMaybe.get();
            }
        }

        return AutoConstants.kEmptyTraj;
    }

    public Pose2d getInitial(Trajectory<SwerveSample> traj, Alliance color) {
        return traj.getInitialPose(color == Alliance.Red).orElse(new Pose2d());
    }

    @Override
    public void onAllianceFound(Alliance color) {

        this.color = color;

        setIsRed();
    }

    public boolean isRed() {

        return this.isRed;
    }

    public void setIsRed() {
        this.isRed = color == Alliance.Red;
    }

    Alliance color;
    boolean isRed = true;
    final SwerveDrive swerve;
    final Superstructure superstructure;
    final Supplier<Twist2d> autoDriveVelocities;

    public final AutoDrive autoDrive;

    public final BooleanSupplier hasTarget;

    public List<AutoMode> redAutosList;
    public List<AutoMode> blueAutosList;

    public final AutoMode blueEmpty;
    public final AutoMode rightMidRushBlue;
    public final AutoMode leftMidRushBlue;
    public final AutoMode outpostBlue;

    public final AutoMode redEmpty;
    public final AutoMode rightMidRushRed;
    public final AutoMode leftMidRushRed;
    public final AutoMode outpostRed;

    public AutoCommands(
            SwerveDrive swerve,
            Superstructure superstructure,
            Supplier<Twist2d> autoDriveVelocities,
            BooleanSupplier hasTarget,
            AutoDrive autoDrive) {
        this.superstructure = superstructure;
        color = Alliance.Red;

        this.autoDrive = autoDrive;

        this.swerve = swerve;
        this.autoDriveVelocities = autoDriveVelocities;

        this.hasTarget = hasTarget;

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
        this.outpostBlue =
                new AutoMode(outpostAuto(), getInitial(getTraj(outpost), Alliance.Blue), outpost);

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
        this.outpostRed =
                new AutoMode(outpostAuto(), getInitial(getTraj(outpost), Alliance.Red), outpost);

        blueAutosList = List.of(blueEmpty, rightMidRushBlue, leftMidRushBlue, outpostBlue);
        redAutosList = List.of(redEmpty, rightMidRushRed, leftMidRushRed, outpostRed);
    }

    public BooleanSupplier hasTarget() {
        return hasTarget;
    }

    public interface ChoreoController extends BiFunction<Pose2d, SwerveSample, ChassisSpeeds> {}
}
