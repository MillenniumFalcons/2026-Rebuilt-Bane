package team3647.frc2026.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import java.util.List;
import team3647.lib.TalonFXSubsystem;

public class Rollers extends TalonFXSubsystem {

    public Rollers(
            TalonFX master,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt) {
        super(List.of(master), velocityConversion, positionConversion, nominalVoltage, kDt);
    }

    public void setOpenLoop(double percent) {
        super.setOpenloop(percent);
    }

    public double getMasterCurrent() {
        return super.getMasterCurrent();
    }

    @Override
    public String getName() {
        return "intake rollers";
    }
}
