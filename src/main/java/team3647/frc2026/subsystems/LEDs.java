package team3647.frc2026.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import team3647.frc2026.constants.LEDConstants;
import team3647.lib.inputs.Joysticks;
import team3647.lib.team9442.AllianceObserver;

public class LEDs implements AllianceObserver {

    private final CANdle candle;

    // Shown when no bound trigger is active. Swapped to the alliance color once known.
    private ControlRequest defaultAnimation = LEDConstants.RAINBOW;

    // The request currently applied to the CANdle; tracked so we only push changes.
    private ControlRequest currentAnimation = defaultAnimation;

    // Highest-priority first. resolve() returns the first active binding's animation.
    private final List<Binding> bindings = new ArrayList<>();

    private record Binding(Trigger trigger, ControlRequest animation) {}

    public LEDs(
            CANdle candle,
            Superstructure superstructure,
            Joysticks mainController,
            BooleanSupplier isBraked) {
        this.candle = candle;

        // Robot is actively firing.
        Trigger shooting = superstructure.shooting;
        // Robot is spinning up the shooter / aligning rotation, not yet ready to fire.
        Trigger spinningUp = superstructure.prepping;
        // Swerve is holding the X-lock (brake).
        Trigger braked = new Trigger(isBraked);
        // Intake is deployed (down) and the rollers are spinning.
        Trigger intaking =
                new Trigger(
                        () ->
                                superstructure.wrist.getAngleDegs() < 5
                                        && Math.abs(superstructure.rollers.getVelocity()) > 40);
        // Intake is stowed while the operator is asking to intake (left bumper held).
        Trigger intakeNotDown =
                new Trigger(() -> superstructure.wrist.angleWithin(0, 5))
                        .and(mainController.leftBumper);

        // Bound highest-priority first.
        bind(braked, LEDConstants.STROBE_RED);
        bind(shooting, LEDConstants.FADE_GREEN);
        bind(intakeNotDown, LEDConstants.STROBE_ORANGE);
        bind(spinningUp, LEDConstants.STROBE_YELLOW);
        bind(intaking, LEDConstants.FADE_PURPLE);
    }

    /**
     * Bind a trigger to an animation. While the trigger is active the LEDs show its animation;
     * earlier-bound triggers win when several are active at once. When no bound trigger is active
     * the LEDs fall back to the default (alliance) animation. Re-evaluated on every trigger edge,
     * so the LEDs revert as soon as a state (shooting, intaking, etc.) ends.
     */
    public void bind(Trigger trigger, ControlRequest animation) {
        bindings.add(new Binding(trigger, animation));
        trigger.onChange(Commands.runOnce(this::update));
    }

    private void update() {
        setAnimation(resolve());
    }

    private ControlRequest resolve() {
        for (Binding binding : bindings) {
            if (binding.trigger().getAsBoolean()) {
                return binding.animation();
            }
        }
        return defaultAnimation;
    }

    public void setAnimation(ControlRequest animation) {
        // if (animation != currentAnimation) {
        //     currentAnimation = animation;
        //     candle.setControl(animation);
        // }
        currentAnimation = animation;
        candle.setControl(animation);
    }

    @Override
    public void onAllianceFound(Alliance color) {
        defaultAnimation = color == Alliance.Red ? LEDConstants.SOLID_RED : LEDConstants.SOLID_BLUE;
        // Re-resolve so that if nothing is active we immediately show the new default.
        update();
    }
}
