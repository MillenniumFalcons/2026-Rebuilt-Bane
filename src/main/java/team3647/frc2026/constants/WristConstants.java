package team3647.frc2026.constants;

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;

public class WristConstants {
    public static final TalonFX kMaster =
            new TalonFX(
                    GlobalConstants.WristIds.kMasterId,
                    new CANBus(GlobalConstants.kSubsystemCanbusName));

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final Angle kMaxAngle = Degree.of(90); // these need changing
    public static final Angle kMinAngle = Degree.of(0);

    public static final double kInitialAngle = kMaxAngle.magnitude();

    public static final double kNativeToDeg =
            0; // this converts rotations to degrees so the units should be degrees/rotation

    static {
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
