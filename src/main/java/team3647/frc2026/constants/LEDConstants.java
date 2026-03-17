// package team3647.frc2026.constants;

// import com.ctre.phoenix6.configs.LEDConfigs;
// import com.ctre.phoenix6.controls.ColorFlowAnimation;
// import com.ctre.phoenix6.controls.FireAnimation;
// import com.ctre.phoenix6.controls.LarsonAnimation;
// import com.ctre.phoenix6.controls.RainbowAnimation;
// import com.ctre.phoenix6.controls.SingleFadeAnimation;
// import com.ctre.phoenix6.controls.SolidColor;
// import com.ctre.phoenix6.controls.StrobeAnimation;
// import com.ctre.phoenix6.controls.TwinkleAnimation;
// import com.ctre.phoenix6.hardware.CANdle;
// import com.ctre.phoenix6.signals.LarsonBounceValue;
// import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
// import com.ctre.phoenix6.signals.RGBWColor;
// import com.ctre.phoenix6.signals.StripTypeValue;

// /** Add your docs here. */
// public class LEDConstants {

//     public static final int CANdleID = 27; //CHANGE
//     public static final CANdle m_candle =
//             new CANdle(LEDConstants.CANdleID, GlobalConstants.kSubsystemCanbusName);

//     public static LEDConfigs config = new LEDConfigs();

//     static {
//         config.BrightnessScalar = 1.0;
//         config.LossOfSignalBehavior = LossOfSignalBehaviorValue.DisableLEDs;
//         config.StripType = StripTypeValue.GRB;
//         m_candle.getConfigurator().apply(config);
//     }

//     // LED Counts
//     public static final int LEDCOUNT = 0; // CHANGE THIS

//     // ANIMATIONS
//     public static final RainbowAnimation RAINBOW =
//             new RainbowAnimation(0, LEDCOUNT)
//                 .withFrameRate(200)
//                 .withBrightness(0.67);

//     public static final FireAnimation FIRE =
//             new FireAnimation(0, LEDCOUNT)
//                 .withFrameRate(50)
//                 .withBrightness(0.67);

//     public static final StrobeAnimation STROBE_ORANGE =
//             new StrobeAnimation(0, LEDCOUNT)
//                 .withColor(new RGBWColor(255, 165, 0))
//                 .withFrameRate(12);

//     public static final StrobeAnimation STROBE_YELLOW =
//             new StrobeAnimation(0, LEDCOUNT)
//                 .withColor(new RGBWColor(255, 255, 0))
//                 .withFrameRate(12);

//     public static final StrobeAnimation STROBE_GREEN =
//             new StrobeAnimation(0, LEDCOUNT)
//                 .withColor(new RGBWColor(0, 255, 0))
//                 .withFrameRate(12);

//     public static final SingleFadeAnimation FADE_GREEN =
//             new SingleFadeAnimation(0, LEDCOUNT)
//                     .withColor(new RGBWColor(0, 255, 0))
//                     .withFrameRate(800);

//     public static final SolidColor SOLID_BLUE =
//             new SolidColor(0, LEDCOUNT)
//                 .withColor(new RGBWColor(0, 255, 0));

//     public static final SolidColor SOLID_RED =
//             new SolidColor(0, LEDCOUNT)
//                 .withColor(new RGBWColor(255, 0, 0));

//     public static final SingleFadeAnimation FADE_PURPLE =
//             new SingleFadeAnimation(0, LEDCOUNT)
//                     .withColor(new RGBWColor(255, 0, 255))
//                     .withFrameRate(200);
//     }
