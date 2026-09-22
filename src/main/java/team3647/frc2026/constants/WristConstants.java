package team3647.frc2026.constants;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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

    public static final Angle kMaxAngle = Degree.of(120); // need to verify physically
    public static final Angle kMinAngle = Degree.of(0);

    public static final Angle kInitialAngle = kMaxAngle;

    // With SensorToMechanismRatio=3 in FeedbackConfigs, TalonFX reports in mechanism rotations
    public static final double kNativeToDeg = 360.0 / 41.67;

    public static final double kCancoderRatio = 3.0; // 3 cancoder rotations = 1 wrist rotation

    // degrees/second
    public static final double maxVel = 360;
    public static final double maxAccel = 600;

    public static final double moi = 1;
    public static final double kV = 0.02;
    public static final double kA =
            0.03 * kV * moi * kNativeToDeg; // moment of inertia around 1 kg m^2
    public static final double kG = kA * 9.81 * 0.25; // 0.25 meters from pivot to center of mass

    // 48T to 16T, 50T to 10T, 50T to 18T
    // 30T to 10T, 50T to 18T, 50T to 10T

    static {
        kMasterConfig.Slot0.withKP(0); // change these values
        kMasterConfig.Slot0.withKI(0);
        kMasterConfig.Slot0.withKD(0);
        kMasterConfig.Slot0.withKV(kV);
        kMasterConfig.Slot0.withKA(kA);
        kMasterConfig.Slot0.withKG(kG);

        kMasterConfig.MotionMagic.MotionMagicCruiseVelocity = maxVel / kNativeToDeg;
        kMasterConfig.MotionMagic.MotionMagicAcceleration = maxAccel / kNativeToDeg;
        kMasterConfig.MotionMagic.MotionMagicExpo_kA = kA * 2;
        kMasterConfig.MotionMagic.MotionMagicExpo_kV = kV * 2;

        kMasterConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        kMasterConfig.Feedback.FeedbackRemoteSensorID = GlobalConstants.WristIds.kWristCancoderId;
        kMasterConfig.Feedback.RotorToSensorRatio = kNativeToDeg / kCancoderRatio;
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

        kCancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        kCancoderConfig.MagnetSensor.MagnetOffset = 0.0; // need to change
        cancoder.getConfigurator().apply(kCancoderConfig);
    }
}
