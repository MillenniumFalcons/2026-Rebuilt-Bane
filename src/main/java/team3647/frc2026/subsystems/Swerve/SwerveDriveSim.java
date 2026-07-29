package team3647.frc2026.subsystems.Swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;
import team3647.lib.vision.Orientation;
import team3647.lib.vision.VisionMeasurement;

public class SwerveDriveSim implements SwerveDrive {
    private final SelfControlledSwerveDriveSimulation simSwerve;

    private final IntakeSimulation intakeSim;

    private PeriodicIOAutoLogged periodicIO = new PeriodicIOAutoLogged();
    private final double kDt;

    public SwerveDriveSim(DriveTrainSimulationConfig config, double kDt) {
        if (!Utils.isSimulation())
            throw new IllegalStateException("TRYING TO RUN SIM DT ON REAL RIO");

        this.simSwerve =
                new SelfControlledSwerveDriveSimulation(
                                new SwerveDriveSimulation(
                                        config, Pose2d.kZero.rotateBy(Rotation2d.k180deg)))
                        .withCurrentLimits(Amps.of(60), Amps.of(60));
        this.kDt = kDt;
        SimulatedArena.getInstance().addDriveTrainSimulation(simSwerve.getDriveTrainSimulation());

        this.intakeSim =
                IntakeSimulation.OverTheBumperIntake(
                        "Fuel",
                        getSim(),
                        Meters.of(0.6604),
                        Meters.of(0.2),
                        IntakeSimulation.IntakeSide.FRONT,
                        30);
    }

    public AbstractDriveTrainSimulation getSim() {
        return this.simSwerve.getDriveTrainSimulation();
    }

    @Override
    public void drive(double x, double y, double rotation) {
        periodicIO.outputSpeeds.vxMetersPerSecond = x;
        periodicIO.outputSpeeds.vyMetersPerSecond = y;
        periodicIO.outputSpeeds.omegaRadiansPerSecond = rotation;

        periodicIO.fieldRelative = false;
    }

    @Override
    public void driveFieldOriented(double x, double y, double rotation) {
        periodicIO.outputSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        x, y, rotation, periodicIO.simPose.getRotation());

        periodicIO.fieldRelative = false;
    }

    public boolean shouldAddData(VisionMeasurement measurement) {
        double distance = measurement.pose.minus(getOdoPose()).getTranslation().getNorm();
        double angle =
                measurement.pose.getRotation().minus(getOdoPose().getRotation()).getDegrees();
        return (distance < MathUtil.clamp(getVelSquared(), 0.25, 1.5) && Math.abs(angle) < 15)
                || DriverStation.isAutonomous();
    }

    private double getVelSquared() {
        return periodicIO.measuredSpeeds.vxMetersPerSecond
                        * periodicIO.measuredSpeeds.vxMetersPerSecond
                + periodicIO.measuredSpeeds.vyMetersPerSecond
                        * periodicIO.measuredSpeeds.vyMetersPerSecond;
    }

    public void addVisionData(VisionMeasurement data) {
        simSwerve.addVisionEstimation(data.pose, data.timestamp, data.stdDevs);
        periodicIO.visionPose = data.pose;
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        simSwerve.addVisionEstimation(pose, timestamp, stdDevs);
        periodicIO.visionPose = pose;
    }

    @Override
    public void writePeriodicOutputs() {
        simSwerve.periodic();
        simSwerve.runChassisSpeeds(
                periodicIO.outputSpeeds.times(1),
                periodicIO.centerOfRotation,
                periodicIO.fieldRelative,
                true);
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.states[0] = simSwerve.getMeasuredStates()[0];
        periodicIO.states[1] = simSwerve.getMeasuredStates()[1];
        periodicIO.states[2] = simSwerve.getMeasuredStates()[2];
        periodicIO.states[3] = simSwerve.getMeasuredStates()[3];

        periodicIO.pose = simSwerve.getOdometryEstimatedPose();
        periodicIO.simPose = simSwerve.getActualPoseInSimulationWorld();
        periodicIO.measuredSpeeds = simSwerve.getMeasuredSpeedsRobotRelative(true);
        periodicIO.actualSpeeds = simSwerve.getActualSpeedsFieldRelative();
        periodicIO.timestamp = Timer.getFPGATimestamp();

        Logger.processInputs(getName(), periodicIO);
    }

    @Override
    public void resetPose(Pose2d pose) {
        simSwerve.resetOdometry(pose);
    }

    @Override
    public void setRobotPose(Pose2d pose) {
        simSwerve.resetOdometry(pose);
        simSwerve.setSimulationWorldPose(pose);
        periodicIO = new PeriodicIOAutoLogged();
    }

    @Override
    public Pose2d getOdoPose() {
        return periodicIO.pose;
    }

    @Override
    public Pose2d getRealPose() {
        return periodicIO.simPose;
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        return periodicIO.measuredSpeeds;
    }

    @Override
    public Orientation getPigeonOrientation() {
        return new Orientation(periodicIO.pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    }

    @Override
    public void onAllianceFound(Alliance color) {
        periodicIO.color = color;
    }

    @Override
    public String getName() {
        return "Swerve Drivetrain";
    }

    @Override
    public Optional<IntakeSimulation> getSimIntake() {
        return Optional.of(this.intakeSim);
    }

    @Override
    public void brake() {
        
    }

    // @Override
    // public Command driveSysIdQuasi(Direction direction) {
    //     // TODO Auto-generated method stub
    //     throw new UnsupportedOperationException("Unimplemented method 'driveSysIdQuasi'");
    // }

    // @Override
    // public Command driveSysIdDynamic(Direction direction) {
    //     // TODO Auto-generated method stub
    //     throw new UnsupportedOperationException("Unimplemented method 'driveSysIdDynamic'");
    // }

    // @Override
    // public Command turnSysIdQuasi(Direction direction) {
    //     // TODO Auto-generated method stub
    //     throw new UnsupportedOperationException("Unimplemented method 'turnSysIdQuasi'");
    // }

    // @Override
    // public Command turnSysIdDynamic(Direction direction) {
    //     // TODO Auto-generated method stub
    //     throw new UnsupportedOperationException("Unimplemented method 'turnSysIdDynamic'");
    // }

    // @Override
    // public Command spinSysIdQuasi(Direction direction) {
    //     // TODO Auto-generated method stub
    //     throw new UnsupportedOperationException("Unimplemented method 'spinSysIdQuasi'");
    // }

    // @Override
    // public Command spinSysIdDynamic(Direction direction) {
    //     // TODO Auto-generated method stub
    //     throw new UnsupportedOperationException("Unimplemented method 'spinSysIdDynamic'");
    // }
}
