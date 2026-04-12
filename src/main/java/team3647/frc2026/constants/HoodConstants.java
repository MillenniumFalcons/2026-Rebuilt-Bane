package team3647.frc2026.constants;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;

public class HoodConstants {
    public static final TalonFX kMaster =
            new TalonFX(
                    GlobalConstants.HoodIds.kMasterId,
                    new CANBus(GlobalConstants.kSubsystemCanbusName));

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    //     public static final CANcoder kCanCoder =
    //             new CANcoder(
    //                     GlobalConstants.HoodIds.kCanCoderId,
    //                     new CANBus(GlobalConstants.kSubsystemCanbusName));

    //     public static final CANcoderConfiguration kCanCoderConfig = new CANcoderConfiguration();

    // CHANGE THESE VALUES
    public static final Angle kMaxAngle = Degree.of(46);
    public static final Angle kMinAngle = Degree.of(1);

    public static final Angle kInitialAngle = Degree.of(0);

    public static final Angle kFeedAngle = Degree.of(33);

    public static final double kNativeToDeg =
            46 / 5.724122; // this converts rotations to degrees so the units should be
    // degrees/rotation
    // divide 360 by gear ratio??
    // 10T to 150T 12T to 36t

    public static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    static {
        // also change these values
        hoodAngleMap.put(1.0, 1.0); // made up
        hoodAngleMap.put(1.5, 1.0);
        hoodAngleMap.put(2.0, 13.0);
        hoodAngleMap.put(2.5, 20.0);
        hoodAngleMap.put(3.0, 21.0);
        hoodAngleMap.put(3.5, 21.0);
        hoodAngleMap.put(4.0, 21.0);
        hoodAngleMap.put(4.5, 30.0); // made up
        hoodAngleMap.put(5.0, 30.0); // made up
        hoodAngleMap.put(6.0, 40.0); // made up
        hoodAngleMap.put(16.0, 40.0);

        kMasterConfig.Slot0.withKP(5.0); // change these values
        kMasterConfig.Slot0.withKI(0);
        kMasterConfig.Slot0.withKD(0);

        kMasterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        kMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                kMaxAngle.in(Degree) / kNativeToDeg;
        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                kMinAngle.in(Degree) / kNativeToDeg;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        kMaster.getConfigurator().apply(kMasterConfig);

        // cancoder configs
        // kCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        // kCanCoderConfig.MagnetSensor.MagnetOffset = 0.015;
        // kCanCoder.getConfigurator().apply(kCanCoderConfig);
    }
}
