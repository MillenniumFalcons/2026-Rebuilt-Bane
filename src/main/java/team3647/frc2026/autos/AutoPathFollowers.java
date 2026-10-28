package team3647.frc2026.autos;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import java.util.function.BiFunction;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import team3647.frc2026.autos.AutoCommands.ChoreoController;
import team3647.frc2026.constants.AutoConstants;
import team3647.frc2026.subsystems.Superstructure;
import team3647.frc2026.subsystems.Swerve.SwerveDrive;
import team3647.frc2026.util.AllianceFlip;
import team3647.frc2026.util.AutoDrive;
import team3647.frc2026.util.AutoDrive.DriveMode;
import team3647.frc2026.util.PoseUtils;
import team3647.lib.team9442.AllianceObserver;

public class AutoPathFollowers implements AllianceObserver {
    private final PIDController rotController = new PIDController(5.0, 0, 0.1);

    final SwerveDrive swerve;
    final Superstructure superstructure;
    final Supplier<Twist2d> autoDriveVelocities;
    public final BooleanSupplier hasTarget;
    public final AutoDrive autoDrive;

    Alliance color;
    boolean isRed = true;

    public AutoPathFollowers(
            SwerveDrive swerve,
            Superstructure superstructure,
            Supplier<Twist2d> autoDriveVelocities,
            BooleanSupplier hasTarget,
            AutoDrive autoDrive) {
        this.swerve = swerve;
        this.superstructure = superstructure;
        this.autoDriveVelocities = autoDriveVelocities;
        this.autoDrive = autoDrive;
        this.hasTarget = hasTarget;
    }

    public Command autoAlign() {
        return Commands.sequence(
                        Commands.runOnce(() -> swerve.drive(0, 0, 0)),
                        autoDrive.enableAutoDrive(),
                        autoDrive.setDriveMode(DriveMode.SHOOT_ON_THE_MOVE),
                        Commands.run(
                                () ->
                                        swerve.driveFieldOriented(
                                                0, 0, -autoDriveVelocities.get().dtheta),
                                swerve))
                .finallyDo(() -> autoDrive.setEnabled(false));
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

    public Command customPathFollowerNi(
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
                () -> PoseUtils.inCircle(poseSupplier.get(), traj.getFinalPose(isRed()).get(), 0.3),
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

            Logger.recordOutput("AutoController/xFF", xFF);
            Logger.recordOutput("AutoController/referenceX", referenceState.getPose().getX());
            Logger.recordOutput("AutoController/yFF", yFF);
            Logger.recordOutput("AutoController/referenceY", referenceState.getPose().getY());
            Logger.recordOutput("Auto/ReferencePose", referenceState.getPose());

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

    public Supplier<Pose2d> getInitialPPLazy(String pathName) {
        return () -> getInitialPP(pathName, DriverStation.getAlliance().orElse(Alliance.Blue));
    }

    public Supplier<Pose2d> getInitialPPMirroredLazy(String pathName) {
        return () ->
                getInitialPPMirrored(pathName, DriverStation.getAlliance().orElse(Alliance.Blue));
    }

    public Pose2d getInitialPP(String pathName, Alliance color) {
        Pose2d startPose;
        try {
            startPose = PathPlannerPath.fromPathFile(pathName).getStartingHolonomicPose().get();
        } catch (Exception e) {
            startPose = new Pose2d();
            e.printStackTrace();
        }
        return AllianceFlip.flip(startPose, color);
    }

    public Pose2d getInitialPPMirrored(String pathName, Alliance color) {
        Pose2d startPose;
        try {
            startPose =
                    PathPlannerPath.fromPathFile(pathName)
                            .mirrorPath()
                            .getStartingHolonomicPose()
                            .get();
        } catch (Exception e) {
            startPose = new Pose2d();
            e.printStackTrace();
        }
        return AllianceFlip.flip(startPose, color);
    }

    @Override
    public void onAllianceFound(Alliance color) {

        this.color = color;

        setIsRed();
    }

    public void setIsRed() {
        this.isRed = color == Alliance.Red;
    }

    public boolean isRed() {
        return this.isRed;
    }

    // =========================================================================
    // PathPlanner path following
    // =========================================================================

    public Command followPathPlannerPath(
            String pathName, BiFunction<Pose2d, Pose2d, Boolean> finished, boolean mirror) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
            if (mirror) {
                path = path.mirrorPath();
            }
            Translation2d finalTranslation =
                    path.getPathPoses().get(path.getPathPoses().size() - 1).getTranslation();
            Rotation2d finalRotation = path.getGoalEndState().rotation();
            Pose2d rawFinalPose = new Pose2d(finalTranslation, finalRotation);
            Command ppCommand = AutoBuilder.followPath(path);
            Timer pathTimer = new Timer();
            BooleanSupplier finishedSupplier =
                    () ->
                            ppCommand.isFinished()
                                    && finished.apply(swerve.getRealPose(), rawFinalPose);
            return new FunctionalCommand(
                    () -> {
                        pathTimer.restart();
                        ppCommand.initialize();
                    },
                    ppCommand::execute,
                    ppCommand::end,
                    finishedSupplier,
                    swerve);
        } catch (Exception e) {
            DriverStation.reportError(
                    "Failed to load PathPlanner path: " + pathName, e.getStackTrace());
            return Commands.none();
        }
    }

    // public Command followPathPlannerPath(String pathName) {
    //     return followPathPlannerPath(pathName, plainFinished(), false);
    // }

    public Command followPathPlannerPath(String pathName) {

        BiFunction<Pose2d, Pose2d, Boolean> finished =
                (finalPose, robotPose) -> {
                    return PoseUtils.aligned(swerve.getOdoPose(), finalPose, 0.1, 10);
                };
        return followPathPlannerPath(pathName, finished, false);
    }

    public Command followMirroredPathPlannerPath(String pathName) {
        BiFunction<Pose2d, Pose2d, Boolean> finished =
                (finalPose, robotPose) -> {
                    return PoseUtils.aligned(swerve.getOdoPose(), finalPose, 0.1, 10);
                };
        return followPathPlannerPath(pathName, finished, true);
    }

    // Odometry-only variants: original finish behavior, no vision required. Kept available in case
    // the vision gate is undesirable for a given path.
    public Command followPathPlannerPathOdoOnly(String pathName) {
        return followPathPlannerPath(pathName, plainFinished(), false);
    }

    public Command followMirroredPathPlannerPathOdoOnly(String pathName) {
        return followPathPlannerPath(pathName, plainFinished(), true);
    }

    /** Original finish: robot simply aligned with the endpoint, no vision required. */
    private BiFunction<Pose2d, Pose2d, Boolean> plainFinished() {
        return (robotPose, endPose) -> PoseUtils.aligned(robotPose, endPose, 0.2, 10);
    }

    /**
     * Finish condition for a PathPlanner path: either the robot is aligned with the endpoint AND a
     * recent vision fix places us near it (so we don't end on drifted odometry), OR — as a fallback
     * — no vision fix has arrived for {@link AutoConstants#kVisionEndpointFallbackSeconds} after
     * the trajectory ended while odometry still places us at the endpoint, so the path doesn't hang
     * forever waiting on vision. The supplier passes {@code (robotPose, endPose)}.
     *
     * <p>This predicate is only evaluated once the trajectory has played out (it's gated behind
     * {@code ppCommand.isFinished()}), so its first evaluation marks the path-end time.
     */
    private BiFunction<Pose2d, Pose2d, Boolean> visionGatedFinished() {
        return new BiFunction<Pose2d, Pose2d, Boolean>() {
            double pathEndTime = -1;
            double lastEval = -1;

            @Override
            public Boolean apply(Pose2d robotPose, Pose2d endPose) {
                double now = Timer.getFPGATimestamp();
                // (Re)start the grace timer on the first eval, or if a gap since the last eval
                // indicates this command is being run again.
                if (pathEndTime < 0 || now - lastEval > 0.5) {
                    pathEndTime = now;
                }
                lastEval = now;

                // Normal completion: aligned + a recent vision fix near the endpoint.
                if (PoseUtils.aligned(robotPose, endPose, 0.2, 360)
                        && swerve.hasRecentVisionNear(
                                endPose,
                                AutoConstants.kVisionEndpointRadiusMeters,
                                AutoConstants.kVisionEndpointMaxAgeSeconds)) {
                    return true;
                }

                // Fallback: no vision for the grace period after the path ended, but odometry
                // still says we're at the endpoint -> end anyway.
                boolean graceElapsed =
                        now - pathEndTime >= AutoConstants.kVisionEndpointFallbackSeconds;
                boolean poseGood = PoseUtils.aligned(robotPose, endPose, 0.5, 360);
                return graceElapsed && poseGood;
            }
        };
    }

    /** Run a full PathPlanner auto by name (loads from deploy/pathplanner/autos/). */
    public Command followPathPlannerAuto(String autoName) {
        try {
            return AutoBuilder.buildAuto(autoName);
        } catch (Exception e) {
            DriverStation.reportError(
                    "Failed to load PathPlanner auto: " + autoName, e.getStackTrace());
            return Commands.none();
        }
    }

    /** Pathfind to a pose using PathPlanner's dynamic pathfinding. */
    public Command pathfindToPose(Pose2d targetPose, PathConstraints constraints) {
        return AutoBuilder.pathfindToPose(targetPose, constraints);
    }

    /** Pathfind to a pose with default constraints. */
    public Command pathfindToPose(Pose2d targetPose) {
        return pathfindToPose(
                targetPose,
                new PathConstraints(3.0, 3.0, Math.toRadians(540), Math.toRadians(720)));
    }
}
