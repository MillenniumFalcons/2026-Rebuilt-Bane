package team3647.frc2026.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class ClimbConstants {
    public static final TalonFX kMaster =
            new TalonFX(
                    GlobalConstants.ClimbIds.kLeftId,
                    new CANBus(GlobalConstants.kSubsystemCanbusName));

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    static {
        kMasterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        kMasterConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        kMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        kMaster.getConfigurator().apply(kMasterConfig);
    }
}
