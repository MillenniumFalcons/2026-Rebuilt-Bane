package team3647.frc2026.util;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import team3647.frc2026.subsystems.Superstructure;

public class LEDTriggers {

    Superstructure superstructure;

    // public final Trigger alignedTrigger = new Trigger(() -> superstructure.isAligned());
    // public final Trigger pieceTrigger = new Trigger(() -> superstructure.hasPeice());
    // public final Trigger intakingTrigger = new Trigger(() -> superstructure.isIntaking());

    public final Trigger alignedTrigger = new Trigger(() -> false);
    public final Trigger pieceTrigger = new Trigger(() -> false);
    public final Trigger intakingTrigger = new Trigger(() -> false);

    // public final Trigger climbingTrigger = new Trigger(() -> );

    public LEDTriggers(Superstructure superstructure) {
        this.superstructure = superstructure;
    }
}
