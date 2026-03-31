package team3647.frc2026.constants;

import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

/** Add your docs here. */
public class LEDConstants {

    public static final int CANdleID = 29; 
    public static final CANdle m_candle =
            new CANdle(LEDConstants.CANdleID, GlobalConstants.kSubsystemCanbusName);

    public static LEDConfigs config = new LEDConfigs();

    static {
        config.BrightnessScalar = 0.67;
        config.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;
        config.StripType = StripTypeValue.RGB;
        m_candle.getConfigurator().apply(config);
    }

    // LED COUNT
    public static final int candleLEDCount = 8; 

    public static final int shooterLEDCount = 50; 

    public static final int hopperLEDCount = 100; 

    public static final int LEDCOUNT = candleLEDCount + 2*shooterLEDCount + hopperLEDCount;

    // ANIMATIONS
    public static final EmptyAnimation EMPTY = new EmptyAnimation(0);

    public static final RainbowAnimation RAINBOW =
            new RainbowAnimation(0, LEDCOUNT).withFrameRate(200).withBrightness(0.67);

    public static final FireAnimation FIRE =
            new FireAnimation(0, LEDCOUNT).withFrameRate(50).withBrightness(0.67);

    public static final StrobeAnimation STROBE_ORANGE =
            new StrobeAnimation(0, LEDCOUNT)
                    .withColor(new RGBWColor(255, 165, 0))
                    .withFrameRate(8);

    public static final StrobeAnimation STROBE_YELLOW =
            new StrobeAnimation(0, LEDCOUNT)
                    .withColor(new RGBWColor(255, 255, 0))
                    .withFrameRate(8);

    public static final StrobeAnimation STROBE_GREEN =
            new StrobeAnimation(0, LEDCOUNT)
                .withColor(new RGBWColor(0, 255, 0))
                .withFrameRate(8);

    public static final SingleFadeAnimation FADE_GREEN =
            new SingleFadeAnimation(0, LEDCOUNT)
                    .withColor(new RGBWColor(0, 255, 0))
                    .withFrameRate(600);

    public static final ColorFlowAnimation FLOW_GREEN =
            new ColorFlowAnimation(0, LEDCOUNT)
                    .withColor(new RGBWColor(0, 255, 0))
                    .withFrameRate(600);

    public static final SolidColor SOLID_BLUE =
            new SolidColor(0, LEDCOUNT).withColor(new RGBWColor(0, 0, 255));

    public static final SolidColor SOLID_RED =
            new SolidColor(0, LEDCOUNT).withColor(new RGBWColor(255, 0, 0));

    public static final SingleFadeAnimation FADE_PURPLE =
            new SingleFadeAnimation(0, LEDCOUNT)
                    .withColor(new RGBWColor(255, 0, 255))
                    .withFrameRate(200);
}
