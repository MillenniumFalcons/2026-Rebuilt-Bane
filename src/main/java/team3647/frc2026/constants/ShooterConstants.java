package team3647.frc2026.constants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final TalonFX kShooterOne =
            new TalonFX(GlobalConstants.ShooterIds.kShooterOneId, GlobalConstants.kSubsystemCanbusName);

    public static final TalonFX kShooterTwo =
            new TalonFX(GlobalConstants.ShooterIds.kShooterTwoId, GlobalConstants.kSubsystemCanbusName);

    public static final TalonFXConfiguration kShooterOneConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kShooterTwoConfig = new TalonFXConfiguration();

    public static final double kGearboxReduction = 1.25; //CHANGE THIS
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.5); //CHANGE THIS
    public static final double kWheelRotationMeters = kWheelDiameterMeters * Math.PI;
    public static final double kNativeVelToSurfaceMpS =
            kWheelRotationMeters / GlobalConstants.kFalconTicksPerRotation * kGearboxReduction;

    static {
        kShooterOneConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        kShooterTwoConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        kShooterOneConfig.Slot0.kP = 0.5;
        kShooterTwoConfig.Slot0.kP = 0.5;

        kShooterOne.getConfigurator().apply(kShooterOneConfig);
        kShooterTwo.getConfigurator().apply(kShooterTwoConfig);

    }
}
