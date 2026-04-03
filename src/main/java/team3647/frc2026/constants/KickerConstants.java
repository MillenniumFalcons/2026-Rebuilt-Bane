package team3647.frc2026.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class KickerConstants {
    // master is bottom right, 1 is top right, 2 is bottom left, 3 is top left (looking from front
    // of robot)

    public static final TalonFX kMaster =
            new TalonFX(
                    GlobalConstants.KickerIds.kMasterId,
                    new CANBus(GlobalConstants.kSubsystemCanbusName));

    public static final TalonFX kSlave1 =
            new TalonFX(
                    GlobalConstants.KickerIds.kSlave1Id,
                    new CANBus(GlobalConstants.kSubsystemCanbusName));

    public static final TalonFX kSlave2 =
            new TalonFX(
                    GlobalConstants.KickerIds.kSlave2Id,
                    new CANBus(GlobalConstants.kSubsystemCanbusName));

    public static final TalonFX kSlave3 =
            new TalonFX(
                    GlobalConstants.KickerIds.kSlave3Id,
                    new CANBus(GlobalConstants.kSubsystemCanbusName));

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    static {
        kMasterConfig.MotorOutput.Inverted =
                InvertedValue.Clockwise_Positive; // might need to be changed

        kMasterConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        kMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        kMaster.getConfigurator().apply(kMasterConfig);
        kSlave1.getConfigurator().apply(kMasterConfig);
        kSlave2.getConfigurator().apply(kMasterConfig);
        kSlave3.getConfigurator().apply(kMasterConfig);
    }
}
