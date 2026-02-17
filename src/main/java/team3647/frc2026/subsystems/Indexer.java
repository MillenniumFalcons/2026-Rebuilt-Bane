package team3647.frc2026.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import java.util.List;
import team3647.lib.TalonFXSubsystem;

public class Indexer extends TalonFXSubsystem {

    public Indexer(
            TalonFX master,
            TalonFX slave,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt) {
        super(List.of(master), velocityConversion, positionConversion, nominalVoltage, kDt);
        this.addFollower(slave, MotorAlignmentValue.Opposed);
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
