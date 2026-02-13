package team3647.frc2026.constants;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;

public class HoodConstants {
    public static final TalonFX kMaster =
            new TalonFX(
                    GlobalConstants.HoodIds.kMasterId,
                    new CANBus(GlobalConstants.kSubsystemCanbusName));

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final Angle kMaxAngle =
            Degree.of(72); // 90-16 (except a little less to make sure it doesnt overdo it)
    public static final Angle kMinAngle = Degree.of(52); // 90-38

    public static final Angle kInitialAngle = kMaxAngle;

    public static final double kNativeToDeg =
            22 / 30.34; // this converts rotations to degrees so the units should be
    // degrees/rotation

    public static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    static {
        // also change these values
        hoodAngleMap.put(1.5, 72.0);
        hoodAngleMap.put(3.0, 72.0);
        hoodAngleMap.put(5.0, 72.0);
        hoodAngleMap.put(7.0, 72.0);
        hoodAngleMap.put(9.0, 72.0);

        kMasterConfig.Slot0.withKP(10); // change these values
        kMasterConfig.Slot0.withKI(0);
        kMasterConfig.Slot0.withKD(0.1);

        kMasterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                kMaxAngle.in(Degree) / kNativeToDeg;
        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                kMinAngle.in(Degree) / kNativeToDeg;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        kMaster.getConfigurator().apply(kMasterConfig);
    }
}
