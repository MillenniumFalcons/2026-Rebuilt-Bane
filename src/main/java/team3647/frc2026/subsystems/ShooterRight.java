package team3647.frc2026.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import team3647.lib.TalonFXSubsystem;

public class ShooterRight extends TalonFXSubsystem {

    public ShooterRight(
            TalonFX motor,
            TalonFX slave,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt) {
        super(motor, velocityConversion, positionConversion, nominalVoltage, kDt);
        this.addFollower(slave, MotorAlignmentValue.Aligned);
    }

    public void setOpenLoop(double percent) {
        super.setOpenloop(percent);
    }

    public double getMasterCurrent() {
        return super.getMasterCurrent();
    }

    public void setVelocity(double velocity) {
        super.setVelocity(velocity, 0);
    }

    @Override
    public void setVoltage(double voltage) {
        super.setVoltage(voltage);
    }

    @Override
    public String getName() {
        return "ShooterRight";
    }
}
