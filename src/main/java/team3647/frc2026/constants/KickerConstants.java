package team3647.frc2026.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

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

    public static final double kP = 10.0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 2.6;

    public static final double kGearboxReduction = 0.25;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(5);
    public static final double kWheelRotationMeters = kWheelDiameterMeters * Math.PI;
    public static final double kNativeVelToSurfaceMpS = kWheelRotationMeters * kGearboxReduction;

    static {
        kMasterConfig.MotorOutput.Inverted =
                InvertedValue.Clockwise_Positive; // might need to be changed

        kMasterConfig.Slot0.kP = kP;
        kMasterConfig.Slot0.kI = kI;
        kMasterConfig.Slot0.kD = kD;
        kMasterConfig.Slot0.kS = kS;

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
