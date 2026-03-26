package team3647.frc2026.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyControl;
import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import team3647.frc2026.constants.LEDConstants;
import team3647.frc2026.util.LEDStates;
import team3647.frc2026.util.LEDStates.SuperstructureState;
import team3647.lib.team6328.VirtualSubsystem;
import team3647.lib.team9442.AllianceObserver;

public class LEDs extends VirtualSubsystem implements AllianceObserver {

    CANdle candle;
    ControlRequest masterRequest = new EmptyControl();
    SuperstructureState currentState;
    LEDStates ledStates;
    ControlRequest defaultAnimation = LEDConstants.SOLID_RED;
    Map<SuperstructureState, ControlRequest> stateAnimations =
            new HashMap<>(
                    Map.of(
                            SuperstructureState.NONE, LEDConstants.SOLID_RED,
                            SuperstructureState.INTAKING, LEDConstants.FADE_PURPLE,
                            SuperstructureState.PREPPING, LEDConstants.STROBE_ORANGE,
                            SuperstructureState.SHOOTING, LEDConstants.FADE_GREEN));

    public LEDs(CANdle candle, LEDStates ledStates) {
        this.candle = candle;
        this.ledStates = ledStates;
        this.currentState = SuperstructureState.NONE;
    }

    public void setAnimation(ControlRequest animation) {
        masterRequest = animation;
    }

    public void setAnimationWithState() {
        SuperstructureState highestState = ledStates.getHighestPriorityState();
        if (currentState != highestState) {
            currentState = highestState;
            masterRequest = stateAnimations.get(currentState);
        } 
        candle.setControl(masterRequest);
        //candle.setControl(masterRequest);
        Logger.recordOutput("led state", currentState);
        Logger.recordOutput("highest state", highestState);
    }

    @Override
    public void periodic() {
        setAnimationWithState();
        ledStates.log();
    }

    @Override
    public void onAllianceFound(Alliance color) {
        if (color == Alliance.Red) {
            defaultAnimation = LEDConstants.SOLID_RED;
        } else {
            defaultAnimation = LEDConstants.SOLID_BLUE;
        }
        stateAnimations.put(SuperstructureState.NONE, defaultAnimation);
    }
}
