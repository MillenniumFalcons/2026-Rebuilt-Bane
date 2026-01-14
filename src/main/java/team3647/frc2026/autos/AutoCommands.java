package team3647.frc2026.autos;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Degrees;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

import team3647.frc2026.Util.AllianceFlip;
import team3647.frc2026.Util.AutoDrive;
import team3647.frc2026.Util.PoseUtils;
import team3647.frc2026.constants.AutoConstants;
import team3647.frc2026.constants.FieldConstants;
import team3647.frc2026.subsystems.Superstructure;
import team3647.frc2026.subsystems.SwerveDrive;
import team3647.lib.team9442.AllianceObserver;

public class AutoCommands implements AllianceObserver {

    private final Rectangle2d BlueLSource =
            new Rectangle2d(
                    new Translation2d(0, FieldConstants.kFieldLengthM), new Translation2d(4, 5.5));
    private final Rectangle2d BlueRSource =
            new Rectangle2d(Translation2d.kZero, new Translation2d(4, 2.5));
    private final Rectangle2d RedLSource =
            new Rectangle2d(
                    AllianceFlip.flip(new Translation2d(0, FieldConstants.kFieldLengthM)),
                    AllianceFlip.flip(new Translation2d(4, 5.5)));
    private final Rectangle2d RedRSource =
            new Rectangle2d(
                    AllianceFlip.flip(Translation2d.kZero),
                    AllianceFlip.flip(new Translation2d(4, 2.5)));

    private final PIDController rotController = new PIDController(3, 0, 0);

    // START OF CHOREO PATH FOLLOWING STUFF

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

    public Command followChoreoPathWithOverrideFast(Trajectory<SwerveSample> traj) {

        // Logger.recordOutput("Autos/current path", path);
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

                            var isInPose =
                                    PoseUtils.inRect(swerve.getOdoPose(), BlueLSource)
                                            || PoseUtils.inRect(swerve.getOdoPose(), BlueRSource)
                                            || PoseUtils.inRect(swerve.getOdoPose(), RedLSource)
                                            || PoseUtils.inRect(swerve.getOdoPose(), RedRSource);
                            if (this.hasTarget.getAsBoolean() && isInPose) {
                                motionXComponent = autoDriveVelocities.get().dx;
                                motionYComponent = autoDriveVelocities.get().dy;
                            }

                            swerve.drive(motionXComponent, motionYComponent, motionRotComponent);
                        },
                        swerve::getOdoPose)
                .andThen(Commands.runOnce(() -> swerve.drive(0, 0, 0), swerve));
    }

    public Command followChoreoPathSlow(Trajectory<SwerveSample> traj) {
        return customPathFollower(
                traj,
                choreoSwerveController(
                        AutoConstants.autoXController,
                        AutoConstants.autoYController,
                        rotController),
                (speeds) -> {
                    swerve.drive(
                            speeds.vxMetersPerSecond * 0.5,
                            speeds.vyMetersPerSecond * 0.5,
                            speeds.omegaRadiansPerSecond * 0.5);
                },
                swerve::getOdoPose);
    }

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

    public Trajectory<SwerveSample> getTraj(String path) {
        var trajMaybe = Choreo.loadTrajectory(path);

        if (trajMaybe.isPresent()) {
            return (Trajectory<SwerveSample>) trajMaybe.get();
        } else {
            return AutoConstants.kEmptyTraj;
        }
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
    SwerveDrive swerve;
    Superstructure superstructure;
    Supplier<Twist2d> autoDriveVelocities;
    Supplier<Twist2d> autoDriveVelocitiesAuto;

    public final AutoDrive autoDrive;

    public final BooleanSupplier hasTarget;

    public List<AutoMode> redAutosList;
    public List<AutoMode> blueAutosList;

    public AutoCommands(
            SwerveDrive swerve,
            Superstructure superstructure,
            Supplier<Twist2d> autoDriveVelocities,
            BooleanSupplier hasTarget,
            AutoDrive autoDrive,
            Supplier<Twist2d> autoDriveVelocitiesAuto) {
        this.superstructure = superstructure;
        color = Alliance.Red;

        this.autoDrive = autoDrive;

        this.autoDriveVelocitiesAuto = autoDriveVelocitiesAuto;
        this.swerve = swerve;
        this.autoDriveVelocities = autoDriveVelocities;

        this.hasTarget = hasTarget;

        blueAutosList =
                List.of(
                        new AutoMode(
                                Commands.none(),
                                Pose2d.kZero.rotateBy(Rotation2d.k180deg),
                                "nothingBlue"));
        redAutosList =
                List.of(
                        new AutoMode(Commands.none(), Pose2d.kZero, "nothingRed"));
    }

    public BooleanSupplier hasTarget() {
        return hasTarget;
    }

    public interface ChoreoController extends BiFunction<Pose2d, SwerveSample, ChassisSpeeds> {}
}
