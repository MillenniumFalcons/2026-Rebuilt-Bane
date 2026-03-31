package team3647.frc2026.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
        kLeftMasterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //change if needed
        kLeftMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        kLeftMasterConfig.CurrentLimits.StatorCurrentLimit = 50.0;
        kLeftMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        kLeftMaster.getConfigurator().apply(kLeftMasterConfig);

        kRightSlaveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // change if needed
        kRightSlaveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        kRightSlaveConfig.CurrentLimits.StatorCurrentLimit = 50.0;
        kRightSlaveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        kRightSlave.getConfigurator().apply(kRightSlaveConfig);
    }
}
