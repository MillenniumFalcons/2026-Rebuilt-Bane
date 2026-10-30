package team3647.frc2026.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import java.util.List;
import team3647.lib.TalonFXSubsystem;

public class Rollers extends TalonFXSubsystem {

    public Rollers(
            TalonFX master,
            TalonFX slave,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt) {
        super(List.of(master), velocityConversion, positionConversion, nominalVoltage, kDt);
        super.addFollower(slave, MotorAlignmentValue.Opposed);
    }

    public void setOpenLoop(double percent) {
        super.setOpenloop(percent);
    }

    public void setVelocityVoltage(double velocity) {
        super.setVelocityVoltage(velocity, 0);
    }

    public void setVelocityTorque(double velocity) {
        super.setVelocityFOC(velocity, 0);
    }

    public double getMasterCurrent() {
        return super.getMasterCurrent();
    }

    @Override
    public String getName() {
        return "Rollers";
    }
}
