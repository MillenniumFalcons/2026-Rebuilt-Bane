package team3647.frc2026.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import team3647.lib.tunables.TunableSlotConfigs;

public class ShooterConstants {
    public static final TalonFX kMaster =
            new TalonFX(
                    GlobalConstants.ShooterIds.kMasterId,
                    new CANBus(GlobalConstants.kSubsystemCanbusName));

    public static final TalonFX kSlave =
            new TalonFX(
                    GlobalConstants.ShooterIds.kSlaveId,
                    new CANBus(GlobalConstants.kSubsystemCanbusName));

    public static final TalonFX kSlave1 =
            new TalonFX(
                    GlobalConstants.ShooterIds.kSlave1Id,
                    new CANBus(GlobalConstants.kSubsystemCanbusName));

    public static final TalonFXConfiguration kShooterConfig = new TalonFXConfiguration();

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final TalonFXConfiguration kSlaveConfig = new TalonFXConfiguration();

    public static final double kGearboxReduction = 0.75; // 18:24 gears
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.5);
    public static final double kWheelRotationMeters = kWheelDiameterMeters * Math.PI;
    public static final double kNativeVelToSurfaceMpS = kWheelRotationMeters * kGearboxReduction;

    public static final double kNominalVoltage = 11.0;
    public static final double kMaxStatorCurrent = 90.0;
    public static final double kMaxSupplyCurrent = 50.0;
    public static final double kidSpeed = 9.0;

    public static final double kIdleSpeed = 8.5;
    public static final double kIdleSpeedMid = 8.0;

    public static final double kS = 4.3;
    public static final double kV = 0.43;
    public static final double kA = 0;

    // these are sysid torquecurrent gains
    //     public static final double kS = 2.0;
    //     public static final double kV = 2.3376;
    //     public static final double kA = 23.006;

    public static final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);

    // need to tune
    public static final double kP = 16.0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final InterpolatingDoubleTreeMap shootSpeedMap = new InterpolatingDoubleTreeMap();

    public static double tempOffset = 0.0;

    static {
        // need to reinterpolate
        shootSpeedMap.put(1.0, 8.5 + tempOffset); // made up
        shootSpeedMap.put(1.5, 8.5 + tempOffset);
        shootSpeedMap.put(2.0, 8.75 + tempOffset);
        shootSpeedMap.put(2.5, 9.25 + tempOffset);
        shootSpeedMap.put(3.0, 9.4 + tempOffset);
        shootSpeedMap.put(3.5, 9.75 + tempOffset);
        shootSpeedMap.put(4.0, 10.5 + tempOffset);
        shootSpeedMap.put(4.5, 11.25 + tempOffset); // made up
        shootSpeedMap.put(5.0, 12.0 + tempOffset); // made up
        shootSpeedMap.put(8.0, 14.5 + tempOffset); // made up
        shootSpeedMap.put(14.0, 14.5);

        kMasterConfig.Slot0.kP = kP;
        kMasterConfig.Slot0.kI = kI;
        kMasterConfig.Slot0.kD = kD;
        kMasterConfig.Slot0.kS = kS;
        kMasterConfig.Slot0.kV = kV;
        kMasterConfig.Slot0.kA = kA;

        // kMasterConfig.Slot1.kP = 999999;

        // kSlaveConfig.Slot0.kP = kP;
        // kSlaveConfig.Slot0.kI = kI;
        // kSlaveConfig.Slot0.kD = kD;
        // kSlaveConfig.Slot0.kS = kS;
        // kSlaveConfig.Slot0.kV = kV;
        // kSlaveConfig.Slot0.kA = kA;

        // kMasterConfig.TorqueCurrent.PeakForwardTorqueCurrent = kMaxCurrent;
        // kMasterConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0;

        kMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // kMasterConfig.MotorOutput.PeakForwardDutyCycle = 1;
        kMasterConfig.MotorOutput.PeakReverseDutyCycle = 0; // might want to remove this
        kMasterConfig.MotorOutput.Inverted =
                InvertedValue.CounterClockwise_Positive; // this should be right

        kMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kMasterConfig.CurrentLimits.StatorCurrentLimit = kMaxStatorCurrent;
        kMasterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        kMasterConfig.CurrentLimits.SupplyCurrentLimit = kMaxSupplyCurrent;

        // kSlaveConfig.TorqueCurrent.PeakForwardTorqueCurrent = kMaxCurrent;
        // kSlaveConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0;

        // kSlaveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // kSlaveConfig.MotorOutput.PeakForwardDutyCycle = 1;
        // kSlaveConfig.MotorOutput.PeakReverseDutyCycle = 0;
        // kSlaveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // kSlaveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        // kSlaveConfig.CurrentLimits.StatorCurrentLimit = kMaxCurrent;

        final TalonFXConfigurator kMasterConfigurator = kMaster.getConfigurator();
        kMasterConfigurator.apply(kMasterConfig);
        final TalonFXConfigurator kSlaveConfigurator = kSlave.getConfigurator();
        kSlaveConfigurator.apply(kMasterConfig);
        final TalonFXConfigurator kSlave1Configurator = kSlave1.getConfigurator();
        kSlave1Configurator.apply(kMasterConfig);

        // final TalonFXConfigurator kSlaveConfigurator = kSlave.getConfigurator();
        // kSlaveConfigurator.apply(kSlaveConfig);

        TunableSlotConfigs tunable = new TunableSlotConfigs(kMaster, "shooter", kMasterConfig.Slot0);
    }
}
