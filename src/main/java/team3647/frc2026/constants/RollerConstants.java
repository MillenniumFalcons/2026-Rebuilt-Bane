package team3647.frc2026.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import team3647.lib.tunables.TunableDouble;
import team3647.lib.tunables.TunableSlotConfigs;

public class RollerConstants {
    public static final TalonFX kMaster =
            new TalonFX(
                    GlobalConstants.RollerIds.kMasterId,
                    new CANBus(GlobalConstants.kSubsystemCanbusName));

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final double kNativeVelToSurfaceMpS = 1; //CHANGE THIS

    public static final double kP = 0.1; //CHANGE THESE
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final Double baseIntakeSpeed = 2.0; //CHANGE THIS
    TunableDouble tunableBaseIntakeSpeed = new TunableDouble("baseIntakeSpeed", baseIntakeSpeed);
    public static final Double intakeSpeedPerRobotSpeed = 1.0; //CHANGE THIS
    TunableDouble tunableIntakeSpeedPerRobotSpeed = new TunableDouble("intakeSpeedScale", intakeSpeedPerRobotSpeed);



    static {
        kMasterConfig.Slot0.kP = kP;
        kMasterConfig.Slot0.kI = kI;
        kMasterConfig.Slot0.kD = kD;

        kMasterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        kMasterConfig.CurrentLimits.StatorCurrentLimit = 55.0;
        kMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        kMaster.getConfigurator().apply(kMasterConfig);
        TunableSlotConfigs tunableConfigs = new TunableSlotConfigs(kMaster, 0, null, null);
    }
}
