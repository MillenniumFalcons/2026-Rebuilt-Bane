package team3647.frc2026.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.util.Units;
import team3647.lib.tunables.TunableSlotConfigs;

public class KickerConstants {
    // master is bottom right, 1 is top right, 2 is bottom left, 3 is top left (looking from front
    // of robot)

    public static final TalonFX kMaster =
            new TalonFX(
                    GlobalConstants.KickerIds.kMasterId,
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
    public static final TalonFXConfiguration kSlaveConfig = new TalonFXConfiguration();

    public static final double kP = 4.5;
    public static final double kI = 0;
    public static final double kD = 0.0;
    public static final double kS = 0.6; // 2.6
    public static final double kV = 0.0;

    public static final double kGearboxReduction = 0.25;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(5);
    public static final double kWheelRotationMeters = kWheelDiameterMeters * Math.PI;
    public static final double kNativeVelToSurfaceMpS = kWheelRotationMeters * kGearboxReduction;

    public static final double kLoadSpeed = 0.5;

    static {
        kMasterConfig.MotorOutput.Inverted =
                InvertedValue.Clockwise_Positive; // might need to be changed

        kMasterConfig.Slot0.kP = kP;
        kMasterConfig.Slot0.kI = kI;
        kMasterConfig.Slot0.kD = kD;
        kMasterConfig.Slot0.kS = kS;
        kMasterConfig.Slot0.kV = kV;

        kMasterConfig.Slot1.kP = 2.0;
        kMasterConfig.Slot1.kI = 6.0;
        kMasterConfig.Slot1.kD = 3.0;
        kMasterConfig.Slot1.kS = 0.0;
        kMasterConfig.Slot1.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        TunableSlotConfigs kTunableSlotConfigs =
                new TunableSlotConfigs(kMaster, "Kicker", kMasterConfig.Slot1);

        kMasterConfig.CurrentLimits.StatorCurrentLimit = 100.0;
        kMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // kSlaveConfig.MotorOutput.Inverted =
        //         InvertedValue.CounterClockwise_Positive; // might need to be changed

        // kSlaveConfig.Slot0.kP = kP;
        // kSlaveConfig.Slot0.kI = kI;
        // kSlaveConfig.Slot0.kD = kD;
        // kSlaveConfig.Slot0.kS = kS;

        // kSlaveConfig.CurrentLimits.StatorCurrentLimit = 60.0;
        // kSlaveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // kSlaveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        kMaster.getConfigurator().apply(kMasterConfig);
        kSlave2.getConfigurator().apply(kMasterConfig);
        kSlave3.getConfigurator().apply(kMasterConfig);
    }
}
