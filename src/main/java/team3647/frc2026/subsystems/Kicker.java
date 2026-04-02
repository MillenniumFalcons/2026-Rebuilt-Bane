package team3647.frc2026.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import java.util.List;
import team3647.lib.TalonFXSubsystem;

public class Kicker extends TalonFXSubsystem {

    public Kicker(
            TalonFX master,
            TalonFX slave1,
            TalonFX slave2,
            TalonFX slave3,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt) {
        super(List.of(master), velocityConversion, positionConversion, nominalVoltage, kDt);
        addFollower(slave1, MotorAlignmentValue.Aligned);
        addFollower(slave2, MotorAlignmentValue.Opposed);
        addFollower(slave3, MotorAlignmentValue.Opposed);
    }

    public void setOpenLoop(double percent) {
        super.setOpenloop(percent);
    }

    public double getMasterCurrent() {
        return super.getMasterCurrent();
    }

    @Override
    public String getName() {
        return "kicker";
    }
}
