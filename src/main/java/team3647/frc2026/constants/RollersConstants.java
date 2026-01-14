package team3647.frc2026.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class RollersConstants {
    public static final TalonFX kMaster =
            new TalonFX(GlobalConstants.RollersIds.kMasterId, GlobalConstants.kSubsystemCanbusName);

    public static final TalonFXConfiguration kmasterConfg = new TalonFXConfiguration();

    static {
        kmasterConfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        kmasterConfg.Slot0.kP = 0.5;

        kMaster.getConfigurator().apply(kmasterConfg);
    }
}
