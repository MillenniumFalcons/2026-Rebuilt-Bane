package team3647.frc2026.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import java.util.List;
import team3647.lib.TalonFXSubsystem;

public class Kicker extends TalonFXSubsystem {

    public Kicker(
            TalonFX master,
            TalonFX slave2,
            TalonFX slave3,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt) {
        super(List.of(master), velocityConversion, positionConversion, nominalVoltage, kDt);
        addFollower(slave2, MotorAlignmentValue.Opposed);
        addFollower(slave3, MotorAlignmentValue.Opposed);
    }

    public void setOpenLoop(double percent) {
        super.setOpenloop(percent);
    }

    public void setVelocityFOC(double velocity) {
        super.setVelocityFOC(velocity, 0);
    }

    public void setVelocityFOC(double velocity, double feedforward) {
        super.setVelocityFOC(velocity, feedforward);
    }

    public void setPosition(double position, double ff) {
        super.setPositionFOC(position, ff, 1);
    }

    public double getMasterCurrent() {
        return super.getMasterCurrent();
    }

    @Override
    public String getName() {
        return "kicker";
    }
}
