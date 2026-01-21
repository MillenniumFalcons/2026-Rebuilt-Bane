package team3647.frc2026.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import team3647.lib.TalonFXSubsystem;

public class ShooterTwo extends TalonFXSubsystem {

    public ShooterTwo(
            TalonFX motor,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt) {
        super(motor, velocityConversion, positionConversion, nominalVoltage, kDt);
    }

    public void setOpenLoop(double percent) {
        super.setOpenloop(percent);
    }

    public double getMasterCurrent() {
        return super.getMasterCurrent();
    }

    @Override
    public void setVelocity(double velocity, double feedforward) {
        super.setVelocity(velocity, feedforward);
    }

    @Override
    public String getName() {
        return "ShooterTwo";
    }
}
