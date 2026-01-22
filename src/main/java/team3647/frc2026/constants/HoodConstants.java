package team3647.frc2026.constants;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;

public class HoodConstants {
    public static final TalonFX kMaster =
            new TalonFX(GlobalConstants.HoodIds.kMasterId, GlobalConstants.kSubsystemCanbusName);

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final Angle kMaxAngle = Degree.of(90);
    public static final Angle kMinAngle = Degree.of(0);

    public static final double kInitialAngle = kMaxAngle.magnitude();

    public static final double kNativeToDeg =
            0; // this converts rotations to degrees so the units should be degrees/rotation

    public static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

    static {
        // also change these values
        hoodAngleMap.put(1.5, 65.0);
        hoodAngleMap.put(3.0, 52.0);
        hoodAngleMap.put(5.0, 40.0);
        hoodAngleMap.put(7.0, 32.0);
        hoodAngleMap.put(9.0, 28.0);

        kMasterConfig.Slot0.withKP(10); // change these values
        kMasterConfig.Slot0.withKI(0);
        kMasterConfig.Slot0.withKD(0.1);

        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                kMaxAngle.in(Degree) / kNativeToDeg;
        kMasterConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                kMinAngle.in(Degree) / kNativeToDeg;
        kMasterConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        kMaster.getConfigurator().apply(kMasterConfig);
    }
}
