package team3647.frc2026.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class KickerConstants {
    //master is bottom right, 1 is top right, 2 is bottom left, 3 is top left (looking from front of robot)

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
        kMasterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // might need to be changed

        kMasterConfig.CurrentLimits.StatorCurrentLimit = 60.0;
        kMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        kMaster.getConfigurator().apply(kMasterConfig);
    }
}
