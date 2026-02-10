package team3647.frc2026.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class IndexerConstants {
    public static final TalonFX kLeftMaster =
            new TalonFX(
                    GlobalConstants.IndexerIds.kIndexerMasterId,
                    new CANBus(GlobalConstants.kSubsystemCanbusName));

    public static final TalonFXConfiguration kLeftMasterConfig = new TalonFXConfiguration();

    public static final TalonFX kRightSlave =
            new TalonFX(
                    GlobalConstants.IndexerIds.kIndexerSlaveId,
                    new CANBus(GlobalConstants.kSubsystemCanbusName));

    public static final TalonFXConfiguration kRightSlaveConfig = new TalonFXConfiguration();

    static {
        kLeftMasterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        kLeftMasterConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        kLeftMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        kLeftMaster.getConfigurator().apply(kLeftMasterConfig);

        kRightSlaveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        kRightSlaveConfig.CurrentLimits.StatorCurrentLimit = 40.0;
        kRightSlaveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        kRightSlave.getConfigurator().apply(kRightSlaveConfig);
    }
}
