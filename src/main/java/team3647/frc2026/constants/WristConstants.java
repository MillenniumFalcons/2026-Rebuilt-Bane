package team3647.frc2026.constants;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;

public class WristConstants {
    public static final TalonFX kMaster =
            new TalonFX(
                    GlobalConstants.WristIds.kMasterId,
                    new CANBus(GlobalConstants.kSubsystemCanbusName));

    public static final CANcoder cancoder =
            new CANcoder(
                    GlobalConstants.WristIds.kWristCancoderId,
                    new CANBus(GlobalConstants.kSubsystemCanbusName));

    public static final CANcoderConfiguration kCancoderConfig = new CANcoderConfiguration();

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final Angle kMaxAngle = Degree.of(129);
    public static final Angle kMinAngle = Degree.of(0);

    public static final Angle kInitialAngle = kMaxAngle;

    public static final double kGearRatio = 41.67; // total rotor-to-wrist gear reduction
    public static final double kCancoderRatio = 3.0; // 3 cancoder rotations = 1 wrist rotation

    // Native units are wrist rotations (mechanism rotations after SensorToMechanismRatio)
    public static final double kNativeToDeg = 360.0;

    // degrees/second
    public static final double maxVel = 540;
    public static final double maxAccel = 640;

    // All gains are in wrist-rotation units (native = wrist rotations)
    public static final double kV = 4.0; // V / (wrist rot/s)
    public static final double kA = 1.5; // V / (wrist rot/s²)
    public static final double kG = 0.4; // V to hold wrist horizontal against gravity

    // 48T to 16T, 50T to 10T, 50T to 18T
    // 30T to 10T, 50T to 18T, 50T to 10T

    static {
        kMasterConfig.Slot0.kP = 160;
        kMasterConfig.Slot0.kI = 0;
        kMasterConfig.Slot0.kD = 8;
        kMasterConfig.Slot0.kV = kV;
        kMasterConfig.Slot0.kA = kA;
        kMasterConfig.Slot0.kG = kG;
        kMasterConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        kMasterConfig.Slot1.kP = 30;
        kMasterConfig.Slot1.kD = 4;
        kMasterConfig.Slot1.kV = kV;
        kMasterConfig.Slot1.kA = kA;
        kMasterConfig.Slot1.kG = kG;
        kMasterConfig.Slot1.GravityType = GravityTypeValue.Arm_Cosine;

        kMasterConfig.MotionMagic.MotionMagicCruiseVelocity = maxVel / kNativeToDeg;
        kMasterConfig.MotionMagic.MotionMagicAcceleration = maxAccel / kNativeToDeg;
        kMasterConfig.MotionMagic.MotionMagicExpo_kV = 2.0;
        kMasterConfig.MotionMagic.MotionMagicExpo_kA = 0.1;

        kMasterConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        kMasterConfig.Feedback.FeedbackRemoteSensorID = GlobalConstants.WristIds.kWristCancoderId;
        kMasterConfig.Feedback.RotorToSensorRatio = kGearRatio / kCancoderRatio;
        kMasterConfig.Feedback.SensorToMechanismRatio = kCancoderRatio;

        kMasterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        kMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        kMasterConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        kMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                kMaxAngle.in(Degree) / kNativeToDeg;
        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                kMinAngle.in(Degree) / kNativeToDeg;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        kMaster.getConfigurator().apply(kMasterConfig);

        kCancoderConfig.MagnetSensor.SensorDirection =
                SensorDirectionValue.CounterClockwise_Positive;
        kCancoderConfig.MagnetSensor.MagnetOffset = -1.0315; // need to change
        kCancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        cancoder.getConfigurator().apply(kCancoderConfig);
    }
}
