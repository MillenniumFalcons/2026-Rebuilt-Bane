package team3647.frc2026.subsystems.Swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveRotation;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveSteerGains;
import com.ctre.phoenix6.swerve.SwerveRequest.SysIdSwerveTranslation;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;
import team3647.frc2026.constants.SwerveDriveConstants;
import team3647.frc2026.constants.TunerConstants.TunerSwerveDrivetrain;
import team3647.lib.ModifiedSignalLogger;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.SwerveFOCRequest;
import team3647.lib.team254.swerve.SwerveKinematicLimits;
import team3647.lib.team254.swerve.SwerveSetpoint;
import team3647.lib.team254.swerve.SwerveSetpointGenerator;
import team3647.lib.team9442.AllianceObserver;
import team3647.lib.vision.Orientation;
import team3647.lib.vision.VisionMeasurement;

// import team3647.lib.vision.VisionMeasurement;

public class SwerveDriveReal extends TunerSwerveDrivetrain
        implements PeriodicSubsystem, AllianceObserver, SwerveDrive {

    public final SwerveSetpointGenerator setpointGenerator;

    public final SwerveKinematicLimits limits;

    public final Field2d field = new Field2d();

    private boolean isChar = false;

    private final double maxSpeedMpS;
    private final double maxRotRadPerSec;

    private PeriodicIOAutoLogged periodicIO = new PeriodicIOAutoLogged();

    private final double kDt;

    private double pitchZero = 0;

    private double cachedSpeed = 0;

    Notifier m_simNotifier;

    // private static final double kSimLoopPeriod = 0.002;

    // real-specific output stuff
    public SwerveRequest masterRequest = new SwerveRequest.Idle();
    public SysIdSwerveRotation spinSysidRequest = new SysIdSwerveRotation();
    public SysIdSwerveTranslation driveVoltageRequest = new SysIdSwerveTranslation();
    public SysIdSwerveSteerGains steerVoltageRequest = new SysIdSwerveSteerGains();
    public SwerveFOCRequest driveFOCRequest = new SwerveFOCRequest(true);
    public SwerveFOCRequest steerFOCRequest = new SwerveFOCRequest(false);
    public FieldCentric fieldCentric =
            new FieldCentric().withDriveRequestType(DriveRequestType.Velocity);
    public RobotCentric robotCentric =
            new RobotCentric()
                    .withDriveRequestType(DriveRequestType.Velocity)
                    .withSteerRequestType(
                            com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType.MotionMagicExpo);

    public SwerveSetpoint setpoint =
            new SwerveSetpoint(
                    new team3647.lib.team254.swerve.ChassisSpeeds(),
                    new team3647.lib.team254.swerve.SwerveModuleState[4]);

    // sysid stuff
    SysIdRoutine driveRoutine =
            new SysIdRoutine(
                    new Config(
                            null,
                            null,
                            Units.Second.of(4),
                            ModifiedSignalLogger.logState("drive routine")),
                    new Mechanism(
                            (Voltage volts) -> setControl(driveVoltageRequest.withVolts(volts)),
                            null,
                            this,
                            "drive routine voltage"));

    SysIdRoutine steerRoutine =
            new SysIdRoutine(
                    new Config(null, null, null, ModifiedSignalLogger.logState("steer routine")),
                    new Mechanism(
                            (Voltage volts) -> setControl(steerVoltageRequest.withVolts(volts)),
                            null,
                            this,
                            "steer routine voltage"));

    SysIdRoutine driveRoutineFOC =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Units.Volts.of(10).per(Units.Second),
                            Units.Volts.of(30),
                            Units.Seconds.of(4),
                            ModifiedSignalLogger.logState("Drive")),
                    new SysIdRoutine.Mechanism(
                            (Voltage volts) ->
                                    setControl(
                                            this.driveFOCRequest.withVoltage(
                                                    volts.in(Units.Volts))),
                            null,
                            this,
                            "Drive motor foc"));

    SysIdRoutine steerRoutineFOC =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Units.Volts.of(3).per(Units.Second),
                            Units.Volts.of(10),
                            Units.Seconds.of(4),
                            ModifiedSignalLogger.logState("steer routine")),
                    new SysIdRoutine.Mechanism(
                            (Voltage volts) ->
                                    setControl(
                                            this.steerFOCRequest.withVoltage(
                                                    volts.in(Units.Volts))),
                            null,
                            this,
                            "steer motor FOC"));

    @SuppressWarnings("unchecked")
    public SwerveDriveReal(
            SwerveDrivetrainConstants swerveDriveConstants,
            double maxSpeedMpS,
            double maxRotRadPerSec,
            double kDt,
            SwerveKinematicLimits limits,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                            ...
                    swerveModuleConstants) {
        super(swerveDriveConstants, swerveModuleConstants);
        registerTelemetry(this::setStuff);

        this.maxSpeedMpS = maxSpeedMpS;
        this.maxRotRadPerSec = maxRotRadPerSec;
        this.kDt = kDt;

        this.setpointGenerator = new SwerveSetpointGenerator(SwerveDriveConstants.kDriveKinematics);

        this.limits = limits;

        this.limits.kMaxDriveVelocity = 5; // TunerConstants.kSpeedAt12VoltsMps;
        this.limits.kMaxDriveAcceleration = 5 / 0.1; // defaultAccel;
        this.limits.kMaxSteeringVelocity = Units.Degree.of(1500).in(Units.Radians);
    }

    public void applyKinematicLimits(SwerveKinematicLimits newLimits) {
        this.limits.kMaxDriveVelocity = newLimits.kMaxDriveVelocity;
        this.limits.kMaxDriveAcceleration = newLimits.kMaxDriveAcceleration;
        this.limits.kMaxSteeringVelocity = newLimits.kMaxSteeringVelocity;
    }

    public void zeroPitch() {
        this.pitchZero = this.getPitch();
    }

    public void readPeriodicInputs() {
        periodicIO.roll = this.getPigeon2().getRoll().getValueAsDouble();
        periodicIO.heading = this.getPigeon2().getYaw().getValueAsDouble();
        periodicIO.pitch = this.getPigeon2().getPitch().getValueAsDouble() - this.pitchZero;
        periodicIO.rawHeading = periodicIO.heading;
        periodicIO.gyroRotation = Rotation2d.fromDegrees(periodicIO.heading);
        periodicIO.timestamp = Timer.getFPGATimestamp();
        periodicIO.frontLeftDriveCurrent =
                this.getModules()[0].getDriveMotor().getSupplyCurrent().getValueAsDouble();
        periodicIO.frontLeftSteerCurrent =
                this.getModules()[0].getSteerMotor().getSupplyCurrent().getValueAsDouble();
        periodicIO.frontRightDriveCurrent =
                this.getModules()[1].getDriveMotor().getSupplyCurrent().getValueAsDouble();
        periodicIO.frontRightSteerCurrent =
                this.getModules()[1].getSteerMotor().getSupplyCurrent().getValueAsDouble();
        periodicIO.backLeftDriveCurrent =
                this.getModules()[2].getDriveMotor().getSupplyCurrent().getValueAsDouble();
        periodicIO.backLeftSteerCurrent =
                this.getModules()[2].getSteerMotor().getSupplyCurrent().getValueAsDouble();
        periodicIO.backRightDriveCurrent =
                this.getModules()[3].getDriveMotor().getSupplyCurrent().getValueAsDouble();
        periodicIO.backRightSteerCurrent =
                this.getModules()[3].getSteerMotor().getSupplyCurrent().getValueAsDouble();
        Logger.processInputs(getName(), periodicIO);
        // SmartDashboard.putBoolean("good", periodicIO.good);

        // SmartDashboard.putNumber("characterization voltage",
        // periodicIO.characterizationVoltage);
    }

    @Override
    public double getTotalDriveCurrent() {
        return periodicIO.frontLeftDriveCurrent
                + periodicIO.frontRightDriveCurrent
                + periodicIO.backLeftDriveCurrent
                + periodicIO.backRightDriveCurrent;
    }

    @Override
    public double getTotalSteerCurrent() {
        return periodicIO.frontLeftSteerCurrent
                + periodicIO.frontRightSteerCurrent
                + periodicIO.backLeftSteerCurrent
                + periodicIO.backRightSteerCurrent;
    }

    public void writePeriodicOutputs() {
        setisAccel();
        if (!isChar) {
            this.setControl(this.masterRequest);
        }
        // Logger.recordOutput("Robot/pose", periodicIO.pose);
        // simpleSim.periodic();

        // SmartDashboard.putNumber("heading", getRawHeading());
    }

    public void periodic() {
        // Logger.recordOutput("is accel", getIsAccel());
        // Logger.recordOutput("Robot/Output", this.getOdoPose());
        // Logger.recordOutput(
        // "Drive/Encoder",
        // this.getModules()[0].getDriveMotor().getPosition().getValueAsDouble());
        // Logger.recordOutput(
        // "Drive/Current",
        // this.getModules()[0].getDriveMotor().getStatorCurrent().getValueAsDouble());

        readPeriodicInputs();
        writePeriodicOutputs();

        // Logger.recordOutput("HEADING", this.getHeading());
    }

    public void setisAccel() {
        if (getVel() > periodicIO.cachedVel + 0.02) {
            periodicIO.isAccel = true;
        } else {
            periodicIO.isAccel = false;
        }
        periodicIO.cachedVel = getVel();
    }

    public Orientation getPigeonOrientation() {
        return new Orientation(periodicIO.pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    }

    public boolean getIsAccel() {
        return periodicIO.isAccel;
    }

    public Command setAccelLimit(double limit) {
        return Commands.runOnce(() -> limits.kMaxDriveAcceleration = limit);
    }

    public void reset() {
        for (int i = 0; i < 4; ++i) {
            this.setpoint.mModuleStates[i] =
                    new team3647.lib.team254.swerve.SwerveModuleState(
                            0.0,
                            team3647.lib.team254.geometry.Rotation2d.fromRadians(
                                    this.getModulePositions()[i].angle.getRadians()));
        }
        periodicIO.good = true;
    }

    public void setRobotPose(Pose2d pose) {
        resetPose(pose);
        periodicIO = new PeriodicIOAutoLogged();
    }

    public void resetEncoders() {
        for (int i = 0; i < 4; i++) {
            this.getModules()[i].getEncoder().setPosition(0);
        }
    }

    public double getTimestamp() {
        return periodicIO.timestamp;
    }

    public void zeroGyro() {
        this.getPigeon2().setYaw(0.0);
    }

    public double getHeading() {
        return periodicIO.heading;
    }

    public double getRoll() {
        return periodicIO.roll;
    }

    public double getPitch() {
        return periodicIO.pitch;
    }

    // private void reduceCancoderStatusframes() {

    // this.backLeft.getCanCoderObject().setStatusFramePeriod(CANCoderStatusFrame.SensorData,
    // 255);
    // this.backRight
    // .getCanCoderObject()
    // .setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
    // this.frontLeft
    // .getCanCoderObject()
    // .setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
    // this.frontRight
    // .getCanCoderObject()
    // .setStatusFramePeriod(CANCoderStatusFrame.SensorData, 255);
    // }

    // // Probably want to moving average filter pitch and roll.
    // public boolean isBalanced(double thresholdDeg) {
    // return Math.abs(getRoll()) < thresholdDeg && Math.abs(getPitch()) <
    // thresholdDeg;
    // }

    public double getRawHeading() {

        return periodicIO.rawHeading;
    }

    public Pose2d getOdoPose() {

        // return RobotBase.isReal()? periodicIO.pose :
        // simpleSim.getOdometryEstimatedPose();
        return periodicIO.pose;
    }

    public void setStuff(SwerveDriveState state) {
        // SignalLogger.writeDoubleArray(
        // "odometry",
        // new double[] {
        // state.Pose.getX(), state.Pose.getY(), state.Pose.getRotation().getDegrees()
        // });
        periodicIO.pose = state.Pose;
        periodicIO.measuredSpeeds = state.Speeds;
        periodicIO.states = state.ModuleStates;
        periodicIO.targets = state.ModuleTargets;

        // SignalLogger.writeDoubleArray(
        // "speeds",
        // new double[] {
        // periodicIO.speeds.vxMetersPerSecond,
        // periodicIO.speeds.vyMetersPerSecond,
        // periodicIO.speeds.omegaRadiansPerSecond
        // });
    }

    public Rotation2d getOdoRot() {
        return getOdoPose().getRotation();
    }

    public double getPoseX() {
        return this.getOdoPose().getX();
    }

    public double getPoseY() {
        return this.getOdoPose().getY();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            this.getModules()[0].getPosition(true),
            this.getModules()[1].getPosition(true),
            this.getModules()[2].getPosition(true),
            this.getModules()[3].getPosition(true)
        };
    }

    public ChassisSpeeds getChassisSpeeds() {
        return periodicIO.measuredSpeeds;
    }

    public double getAccel() {
        double accel = (getChassisSpeeds().vxMetersPerSecond - cachedSpeed) / 0.02;
        this.cachedSpeed = getChassisSpeeds().vxMetersPerSecond;
        return accel;
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {

        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getOdoRot());
    }

    public boolean shouldAddData(VisionMeasurement measurement) {
        double distance = measurement.pose.minus(getOdoPose()).getTranslation().getNorm();
        double angle =
                measurement.pose.getRotation().minus(getOdoPose().getRotation()).getDegrees();
        // System.out.println(
        //         (distance < MathUtil.clamp(getVel(), 0.25, 1) && Math.abs(angle) < 15)
        //                 || DriverStation.isAutonomous());
        return (distance < MathUtil.clamp(getVel(), 0.25, 1) && Math.abs(angle) < 15)
                || DriverStation.isAutonomous();
    }

    public void addVisionData(VisionMeasurement data) {
        periodicIO.visionPose = data.pose;
        addVisionMeasurement(data.pose, Utils.fpgaToCurrentTime(data.timestamp), data.stdDevs);
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        periodicIO.visionPose = pose;
        super.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    public void onAllianceFound(Alliance color) {
        periodicIO.color = color;

        // var rot = color == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero;

        // setOperatorPerspectiveForward(rot);
    }

    public void end() {
        stopModules();
    }

    public void stopModules() {
        this.masterRequest = new SwerveRequest.Idle();
    }

    public void drive(double x, double y, double rotation) {

        if (!periodicIO.good) {
            reset();
            return;
        }
        SwerveSetpoint setpoint = generateRobotOriented(x, y, -rotation);
        this.robotCentric
                .withVelocityX(setpoint.mChassisSpeeds.vxMetersPerSecond)
                .withVelocityY(setpoint.mChassisSpeeds.vyMetersPerSecond)
                .withRotationalRate(setpoint.mChassisSpeeds.omegaRadiansPerSecond);
        this.masterRequest = this.robotCentric;
    }

    public SwerveSetpoint generate(double x, double y, double omega) {
        Logger.recordOutput("prevspt", this.setpoint.mChassisSpeeds.vyMetersPerSecond);
        var robotRel =
                team3647.lib.team254.swerve.ChassisSpeeds.fromFieldRelativeSpeeds(
                        x,
                        y,
                        omega,
                        team3647.lib.team254.geometry.Rotation2d.fromDegrees(
                                this.getOdoPose().getRotation().getDegrees()));

        this.setpoint = setpointGenerator.generateSetpoint(limits, this.setpoint, robotRel, kDt);

        return this.setpoint;
    }

    public SwerveSetpoint generateRobotOriented(double x, double y, double omega) {
        var robotRel = new team3647.lib.team254.swerve.ChassisSpeeds(x, y, omega);
        this.setpoint = setpointGenerator.generateSetpoint(limits, this.setpoint, robotRel, kDt);
        return this.setpoint;
    }

    public void driveNative(double x, double y, double rotation) {
        this.robotCentric.withVelocityX(x).withVelocityY(y).withRotationalRate(rotation);
    }

    // public void driveFieldOriented(double x, double y, double rotation) {
    //     if (!periodicIO.good) {
    //         reset();
    //         return;
    //     }
    //     SwerveSetpoint setpoint = generate(x, y, -rotation);
    //     this.robotCentric
    //             .withVelocityX(setpoint.mChassisSpeeds.vxMetersPerSecond)
    //             .withVelocityY(setpoint.mChassisSpeeds.vyMetersPerSecond)
    //             .withRotationalRate(setpoint.mChassisSpeeds.omegaRadiansPerSecond);

    //     this.masterRequest = this.robotCentric;
    // }

    public void driveFieldOriented(double x, double y, double rotation) {
        this.fieldCentric
                .withVelocityX(x)
                .withVelocityY(y)
                .withRotationalRate(rotation);
        this.masterRequest = this.fieldCentric;
    }

    // public void resetSimOdo(){
    // simpleSim.resetOdometry(simpleSim.getActualPoseInSimulationWorld());
    // }

    public void driveFieldOriented(DoubleSupplier x, double y, double rotation) {
        if (!periodicIO.good) {
            reset();
            return;
        }
        SwerveSetpoint setpoint = generate(x.getAsDouble(), y, -rotation);
        this.robotCentric
                .withVelocityX(setpoint.mChassisSpeeds.vxMetersPerSecond)
                .withVelocityY(setpoint.mChassisSpeeds.vyMetersPerSecond)
                .withRotationalRate(setpoint.mChassisSpeeds.omegaRadiansPerSecond);
        this.masterRequest = this.robotCentric;
    }

    public void driveFieldOriented(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
        if (!periodicIO.good) {
            reset();
            return;
        }
        SwerveSetpoint setpoint =
                generate(x.getAsDouble(), y.getAsDouble(), -rotation.getAsDouble());
        this.robotCentric
                .withVelocityX(setpoint.mChassisSpeeds.vxMetersPerSecond)
                .withVelocityY(setpoint.mChassisSpeeds.vyMetersPerSecond)
                .withRotationalRate(setpoint.mChassisSpeeds.omegaRadiansPerSecond);
        this.masterRequest = this.robotCentric;
    }

    @Override
    public void brake() {
        this.masterRequest = new SwerveRequest.SwerveDriveBrake();
    }

    public SwerveModuleState[] getModuleStates() {
        return periodicIO.states;
    }

    public double getVel() { // squared
        return this.getChassisSpeeds().vxMetersPerSecond * this.getChassisSpeeds().vxMetersPerSecond
                + this.getChassisSpeeds().vyMetersPerSecond
                        * this.getChassisSpeeds().vyMetersPerSecond;
    }

    public double getMaxSpeedMpS() {
        return this.maxSpeedMpS;
    }

    public double getMaxRotationRadpS() {
        return this.maxRotRadPerSec;
    }

    public String getName() {
        return "Swerve Drivetrain";
    }

    public Command driveRoutineQuasiStatic() {
        return driveRoutine
                .quasistatic(Direction.kForward)
                .beforeStarting(() -> this.isChar = true)
                .finallyDo(() -> this.isChar = false);
    }

    public Command driveRoutineDynamic() {
        return driveRoutine
                .dynamic(Direction.kForward)
                .beforeStarting(() -> this.isChar = true)
                .finallyDo(() -> this.isChar = false);
    }

    public Command driveRoutineQuasiStaticReverse() {
        return driveRoutine
                .quasistatic(Direction.kReverse)
                .beforeStarting(() -> this.isChar = true)
                .finallyDo(() -> this.isChar = false);
    }

    public Command driveRoutineDynamicReverse() {
        return driveRoutine
                .dynamic(Direction.kReverse)
                .beforeStarting(() -> this.isChar = true)
                .finallyDo(() -> this.isChar = false);
    }

    public Command steerRoutineQuasiStatic() {
        return steerRoutine.quasistatic(Direction.kForward);
    }

    public Command steerRoutineDynamic() {
        return steerRoutine.dynamic(Direction.kForward);
    }

    @Override
    public Pose2d getRealPose() {
        return periodicIO.pose;
    }

    @Override
    public Optional<IntakeSimulation> getSimIntake() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSimIntake'");
    }

    @Override
    public AbstractDriveTrainSimulation getSim() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getSim'");
    }
}
