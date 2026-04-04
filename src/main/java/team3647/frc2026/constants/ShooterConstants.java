package team3647.frc2026.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final TalonFX kMaster =
            new TalonFX(
                    GlobalConstants.ShooterIds.kMasterId,
                    new CANBus(GlobalConstants.kSubsystemCanbusName));

    public static final TalonFX kSlave =
            new TalonFX(
                    GlobalConstants.ShooterIds.kSlaveId,
                    new CANBus(GlobalConstants.kSubsystemCanbusName));

    public static final TalonFXConfiguration kShooterConfig = new TalonFXConfiguration();

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final TalonFXConfiguration kSlaveConfig = new TalonFXConfiguration();

    public static final Transform2d robotToBetweenShooter =
            new Transform2d(0, 0.25, Rotation2d.kZero);

    public static final double kGearboxReduction = 0.75; // 18:24 gears
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.5);
    public static final double kWheelRotationMeters = kWheelDiameterMeters * Math.PI;
    public static final double kNativeVelToSurfaceMpS =
            kWheelRotationMeters / GlobalConstants.kFalconTicksPerRotation * kGearboxReduction;

    public static final double kNominalVoltage = 11.0;
    public static final double kMaxCurrent = 90.0;

    // these are sysid voltage gains
    public static final double kS = 0.015988;
    public static final double kV = 0.13528;
    public static final double kA = 0.092748;

    // these are sysid torquecurrent gains
    //     public static final double kS = 2.0;
    //     public static final double kV = 2.3376;
    //     public static final double kA = 23.006;

    public static final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);

    // need to tune
    public static final double kP = 0.5;
    public static final double kI = 0;
    public static final double kD = 0.0025;

    public static final InterpolatingDoubleTreeMap shootSpeedMap = new InterpolatingDoubleTreeMap();

    public static double tempOffset = 0;

    static {
        // need to reinterpolate
        shootSpeedMap.put(1.0, 13.5 + tempOffset);
        shootSpeedMap.put(1.5, 14.0 + tempOffset);
        shootSpeedMap.put(2.0, 15.0 + tempOffset);
        shootSpeedMap.put(2.5, 15.75 + tempOffset);
        shootSpeedMap.put(3.0, 16.0 + tempOffset);
        shootSpeedMap.put(3.5, 17.0 + tempOffset);
        shootSpeedMap.put(4.0, 18.0 + tempOffset);
        shootSpeedMap.put(4.5, 18.75 + tempOffset);
        shootSpeedMap.put(5.0, 19.25 + tempOffset);
        shootSpeedMap.put(5.5, 20.0 + tempOffset);
        shootSpeedMap.put(6.0, 20.5 + tempOffset);
        shootSpeedMap.put(6.5, 21.0 + tempOffset);
        shootSpeedMap.put(7.0, 21.5 + tempOffset);
        shootSpeedMap.put(7.5, 22.0 + tempOffset);
        shootSpeedMap.put(8.0, 22.5 + tempOffset);

        kMasterConfig.Slot0.kP = kP;
        kMasterConfig.Slot0.kI = kI;
        kMasterConfig.Slot0.kD = kD;
        kMasterConfig.Slot0.kS = kS;
        kMasterConfig.Slot0.kV = kV;
        kMasterConfig.Slot0.kA = kA;

        // kMasterConfig.Slot1.kP = 999999;

        kSlaveConfig.Slot0.kP = kP;
        kSlaveConfig.Slot0.kI = kI;
        kSlaveConfig.Slot0.kD = kD;
        kSlaveConfig.Slot0.kS = kS;
        kSlaveConfig.Slot0.kV = kV;
        kSlaveConfig.Slot0.kA = kA;

        // kMasterConfig.TorqueCurrent.PeakForwardTorqueCurrent = kMaxCurrent;
        // kMasterConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0;

        kMasterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // kMasterConfig.MotorOutput.PeakForwardDutyCycle = 1;
        kMasterConfig.MotorOutput.PeakReverseDutyCycle = 0; // might want to remove this
        kMasterConfig.MotorOutput.Inverted =
                InvertedValue.CounterClockwise_Positive; // this should be right

        kMasterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kMasterConfig.CurrentLimits.StatorCurrentLimit = kMaxCurrent;

        // kSlaveConfig.TorqueCurrent.PeakForwardTorqueCurrent = kMaxCurrent;
        // kSlaveConfig.TorqueCurrent.PeakReverseTorqueCurrent = 0;

        kSlaveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        // kSlaveConfig.MotorOutput.PeakForwardDutyCycle = 1;
        kSlaveConfig.MotorOutput.PeakReverseDutyCycle = 0;
        kSlaveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        kSlaveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        kSlaveConfig.CurrentLimits.StatorCurrentLimit = kMaxCurrent;

        final TalonFXConfigurator kMasterConfigurator = kMaster.getConfigurator();
        kMasterConfigurator.apply(kMasterConfig);

        final TalonFXConfigurator kSlaveConfigurator = kSlave.getConfigurator();
        kSlaveConfigurator.apply(kSlaveConfig);
    }
}

// OLD SHOOTER CODE IN CASE WE WANT TO USE BETA

// public class ShooterConstants {
//     // one is left two is right i got too lazy to change
//     public static final TalonFX kShooterOne =
//             new TalonFX(
//                     GlobalConstants.ShooterIds.kShooterOneId,
//                     new CANBus(GlobalConstants.kSubsystemCanbusName));

//     public static final TalonFX kShooterOneSlave =
//             new TalonFX(
//                     GlobalConstants.ShooterIds.kShooterOneSlaveId,
//                     new CANBus(GlobalConstants.kSubsystemCanbusName));

//     public static final TalonFX kShooterTwo =
//             new TalonFX(
//                     GlobalConstants.ShooterIds.kShooterTwoId,
//                     new CANBus(GlobalConstants.kSubsystemCanbusName));

//     public static final TalonFX kShooterTwoSlave =
//             new TalonFX(
//                     GlobalConstants.ShooterIds.kShooterTwoSlaveId,
//                     new CANBus(GlobalConstants.kSubsystemCanbusName));

//     public static final TalonFXConfiguration kShooterOneConfig = new TalonFXConfiguration();
//     public static final TalonFXConfiguration kShooterTwoConfig = new TalonFXConfiguration();

//     public static final Transform2d robotToBetweenShooter = new Transform2d();

//     public static final double kGearboxReduction = 1; // i think this is correct now
//     public static final double kWheelDiameterMeters =
//             Units.inchesToMeters(4); // i think this is also correct now
//     public static final double kWheelRotationMeters = kWheelDiameterMeters * Math.PI;
//     public static final double kNativeVelToSurfaceMpS =
//             kWheelRotationMeters / GlobalConstants.kFalconTicksPerRotation * kGearboxReduction;

//     public static final double kNominalVoltage = 11.0;
//     public static final double kMaxCurrent = 70.0;

//     // feedforward from sysid left shooter
//     public static final double kS = 0.0;
//     public static final double kV = 0.12;
//     public static final double kA = 0.0098105;

//     // sysid right shooter
//     public static final double kSright = 0.0;
//     public static final double kVright = 0.123;
//     public static final double kAright = 0.0098557;

//     public static final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);

//     // pid
//     public static final double shooterOnekP = 0.5;
//     public static final double shooterTwokP = 0.5;
//     public static final double masterkI = 0;
//     public static final double masterkD = 0.0025;

//     public static final InterpolatingDoubleTreeMap shootSpeedMap = new
// InterpolatingDoubleTreeMap();

//     public static double tempOffset = 0;

//     static {
//         shootSpeedMap.put(1.0, 13.5 + tempOffset);
//         shootSpeedMap.put(1.5, 14.0 + tempOffset);
//         shootSpeedMap.put(2.0, 15.0 + tempOffset);
//         shootSpeedMap.put(2.5, 15.75 + tempOffset);
//         shootSpeedMap.put(3.0, 16.0 + tempOffset);
//         shootSpeedMap.put(3.5, 17.0 + tempOffset);
//         shootSpeedMap.put(4.0, 18.0 + tempOffset);
//         shootSpeedMap.put(4.5, 18.75 + tempOffset);
//         shootSpeedMap.put(5.0, 19.25 + tempOffset);
//         shootSpeedMap.put(5.5, 20.0 + tempOffset);
//         shootSpeedMap.put(6.0, 20.5 + tempOffset);
//         shootSpeedMap.put(6.5, 21.0 + tempOffset);
//         shootSpeedMap.put(7.0, 21.5 + tempOffset);
//         shootSpeedMap.put(7.5, 22.0 + tempOffset);
//         shootSpeedMap.put(8.0, 22.5 + tempOffset);

//         kShooterOneConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
//         kShooterTwoConfig.MotorOutput.Inverted =
//                 InvertedValue.CounterClockwise_Positive; // these should be opposites of each
// other

//         MotorOutputConfigs shooterOneMotor = new MotorOutputConfigs();
//         MotorOutputConfigs shooterTwoMotor = new MotorOutputConfigs();

//         CurrentLimitsConfigs shooterOneCurrent = new CurrentLimitsConfigs();
//         CurrentLimitsConfigs shooterTwoCurrent = new CurrentLimitsConfigs();

//         // shooter one
//         kShooterOneConfig.Slot0.kP = shooterOnekP;
//         kShooterOneConfig.Slot0.kI = masterkI;
//         kShooterOneConfig.Slot0.kD = masterkD;
//         kShooterOneConfig.Slot0.kS = kS;
//         kShooterOneConfig.Slot0.kV = kV;
//         kShooterOneConfig.Slot0.kA = kA;

//         shooterOneMotor.NeutralMode = NeutralModeValue.Coast;
//         shooterOneMotor.PeakReverseDutyCycle = 0;
//         shooterOneMotor.Inverted = InvertedValue.Clockwise_Positive;

//         shooterOneCurrent.StatorCurrentLimitEnable = true;
//         shooterOneCurrent.StatorCurrentLimit = kMaxCurrent;

//         // shooter two
//         kShooterTwoConfig.Slot0.kP = shooterTwokP;
//         kShooterTwoConfig.Slot0.kI = masterkI;
//         kShooterTwoConfig.Slot0.kD = masterkD;
//         kShooterTwoConfig.Slot0.kS = kSright;
//         kShooterTwoConfig.Slot0.kV = kVright;
//         kShooterTwoConfig.Slot0.kA = kAright;

//         shooterTwoMotor.NeutralMode = NeutralModeValue.Coast;
//         shooterTwoMotor.PeakReverseDutyCycle = 0;
//         shooterTwoMotor.Inverted = InvertedValue.CounterClockwise_Positive;

//         shooterTwoCurrent.StatorCurrentLimitEnable = true;
//         shooterTwoCurrent.StatorCurrentLimit = kMaxCurrent;

//         // apply configs
//         final TalonFXConfigurator kShooterOneConfigurator = kShooterOne.getConfigurator();
//         final TalonFXConfigurator kShooterTwoConfigurator = kShooterTwo.getConfigurator();
//         kShooterOneConfigurator.apply(kShooterOneConfig);
//         kShooterOneConfigurator.apply(shooterOneCurrent);
//         kShooterOneConfigurator.apply(shooterOneMotor);

//         kShooterTwoConfigurator.apply(kShooterTwoConfig);
//         kShooterTwoConfigurator.apply(shooterTwoCurrent);
//         kShooterTwoConfigurator.apply(shooterTwoMotor);

//         final TalonFXConfigurator kShooterOneSlaveConfigurator =
// kShooterOneSlave.getConfigurator();
//         final TalonFXConfigurator kShooterTwoSlaveConfigurator =
// kShooterTwoSlave.getConfigurator();
//         kShooterOneSlaveConfigurator.apply(kShooterOneConfig);
//         kShooterOneSlaveConfigurator.apply(shooterOneCurrent);
//         kShooterOneSlaveConfigurator.apply(shooterOneMotor);

//         kShooterTwoSlaveConfigurator.apply(kShooterTwoConfig);
//         kShooterTwoSlaveConfigurator.apply(shooterTwoCurrent);
//         kShooterTwoSlaveConfigurator.apply(shooterTwoMotor);
//     }
// }
