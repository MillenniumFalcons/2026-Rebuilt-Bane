package team3647.frc2026.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class RollerConstants {
    public static final TalonFX kMaster =
            new TalonFX(
                    GlobalConstants.RollerIds.kMasterId,
                    new CANBus(GlobalConstants.kSubsystemCanbusName));

    public static final TalonFX kSlave =
            new TalonFX(
                    GlobalConstants.RollerIds.kSlaveId,
                    new CANBus(GlobalConstants.kSubsystemCanbusName));

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final double kNativeToSurfaceRPS = 1;

    // Intake speed scaling with drivetrain forward velocity.
    // Final intake speed = kIntakeBaseSpeed + kIntakeVxGain * max(0, vx_m_s)
    public static final double kIntakeBaseSpeed = 75.0;
    public static final double kIntakeVxGain = 3.0;

    public static final double kP = 15.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    static {
        kMasterConfig.Slot0.kP = kP;
        kMasterConfig.Slot0.kI = kI;
        kMasterConfig.Slot0.kD = kD;

        kMasterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        kMasterConfig.CurrentLimits.SupplyCurrentLimit = 50.0;
        kMasterConfig.CurrentLimits.StatorCurrentLimit = 90.0;
        kMasterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        kMaster.getConfigurator().apply(kMasterConfig);
        kSlave.getConfigurator()
                .apply(
                        kMasterConfig.withMotorOutput(
                                new MotorOutputConfigs()
                                        .withInverted(InvertedValue.Clockwise_Positive)));
    }
}
