// package team3647.frc2026.subsystems;
// import com.ctre.phoenix6.controls.ControlRequest;
// import com.ctre.phoenix6.controls.EmptyControl;
// import com.ctre.phoenix6.hardware.CANdle;

// import edu.wpi.first.wpilibj.DriverStation.Alliance;

// import java.util.Map;
// import java.util.ResourceBundle.Control;
// import java.util.function.Supplier;
// import team3647.frc2026.constants.LEDConstants;
// import team3647.frc2026.subsystems.Superstructure.SuperstructureState;
// import team3647.lib.team6328.VirtualSubsystem;
// import team3647.lib.team9442.AllianceObserver;

// public class LEDs extends VirtualSubsystem implements AllianceObserver{

//     CANdle candle;
//     ControlRequest masterRequest = new EmptyControl();
//     Supplier<SuperstructureState> stateSupplier;
//     ControlRequest defaultAnimation = LEDConstants.SOLID_RED;

//     Map<SuperstructureState, ControlRequest> stateAnimations =
//             Map.of(
//                     SuperstructureState.NONE, defaultAnimation,
//                     SuperstructureState.INTAKING, LEDConstants.FADE_PURPLE,
//                     SuperstructureState.PREPPING, LEDConstants.STROBE_ORANGE,
//                     SuperstructureState.SHOOTING, LEDConstants.FADE_GREEN
//                     );

//     public LEDs(CANdle candle, Supplier<SuperstructureState> stateSupplier) {
//         this.candle = candle;
//         this.stateSupplier = stateSupplier;
        
//     }

//     public void setAnimation(ControlRequest animation) {
//         masterRequest = animation;
//     }

//     public void setAnimationWithState() {
//         SuperstructureState state = stateSupplier.get();
//         if (state == null) {
//             state = SuperstructureState.NONE;
//         }
//         masterRequest = stateAnimations.get(state);
//     }

//     @Override
//     public void periodic() {
//         setAnimationWithState();
//         candle.setControl(masterRequest);
//     }

//     @Override
//     public void onAllianceFound(Alliance color) {
//         if (color == Alliance.Red) {
//             defaultAnimation = LEDConstants.SOLID_RED;
//         } else {
//             defaultAnimation = LEDConstants.SOLID_BLUE;
//         }
//     }
// }