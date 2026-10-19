package team3647.frc2026.subsystems.Swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.AutoLog;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.team9442.AllianceObserver;
import team3647.lib.vision.Orientation;
import team3647.lib.vision.VisionMeasurement;

public interface SwerveDrive extends PeriodicSubsystem, AllianceObserver {

    public void drive(double x, double y, double rotation);

    public void driveFieldOriented(double x, double y, double rotation);

    public void brake();

    public void resetPose(Pose2d pose);

    public void setRobotPose(Pose2d pose);

    // public Command driveSysIdQuasi(SysIdRoutine.Direction direction);

    // public Command driveSysIdDynamic(SysIdRoutine.Direction direction);

    // public Command turnSysIdQuasi(SysIdRoutine.Direction direction);

    // public Command turnSysIdDynamic(SysIdRoutine.Direction direction);

    // public Command spinSysIdQuasi(SysIdRoutine.Direction direction);

    // public Command spinSysIdDynamic(SysIdRoutine.Direction direction);

    public Pose2d getOdoPose();

    public Pose2d getRealPose();

    public ChassisSpeeds getChassisSpeeds();

    public Orientation getPigeonOrientation();

    public void addVisionData(VisionMeasurement data);

    public boolean shouldAddData(VisionMeasurement data);

    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs);

    /**
     * When enabled, the pose estimator trusts vision almost absolutely and effectively ignores
     * wheel odometry. Use while the robot is beached/high-centered and the wheels are slipping.
     */
    default void setVisionOnlyMode(boolean enabled) {}

    public Optional<IntakeSimulation> getSimIntake();

    public AbstractDriveTrainSimulation getSim();

    default double getTotalDriveCurrent() {
        return 0;
    }

    default double getTotalSteerCurrent() {
        return 0;
    }

    @AutoLog
    public static class PeriodicIO {
        // inputs

        public boolean good = false;

        public double cachedVel = 0;
        public boolean isAccel = false;

        public double characterizationVoltage = 0;
        public boolean isOpenloop = true;
        public double heading = 0;
        public double roll = 0;
        public double pitch = 0;
        public double rawHeading = 0;
        public Rotation2d gyroRotation = new Rotation2d();

        public Alliance color = Alliance.Blue;

        public SwerveModuleState[] states =
                new SwerveModuleState[] {
                    new SwerveModuleState(), // FL
                    new SwerveModuleState(), // FR
                    new SwerveModuleState(), // BL
                    new SwerveModuleState() // BR
                };

        public SwerveModuleState[] targets =
                new SwerveModuleState[] {
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState()
                };

        public double timestamp = 0;

        public Pose2d visionPose = new Pose2d();

        public Pose2d pose = new Pose2d();
        public ChassisSpeeds measuredSpeeds = new ChassisSpeeds();

        // sim-specific stuff
        public ChassisSpeeds outputSpeeds = new ChassisSpeeds();
        public boolean fieldRelative = true;
        public Translation2d centerOfRotation = Translation2d.kZero;
        public Pose2d simPose = new Pose2d(2, 2, new Rotation2d());
        public ChassisSpeeds actualSpeeds = new ChassisSpeeds();

        public double frontLeftDriveCurrent = 0;
        public double frontLeftSteerCurrent = 0;
        public double frontRightDriveCurrent = 0;
        public double frontRightSteerCurrent = 0;
        public double backLeftDriveCurrent = 0;
        public double backLeftSteerCurrent = 0;
        public double backRightDriveCurrent = 0;
        public double backRightSteerCurrent = 0;
    }
}
