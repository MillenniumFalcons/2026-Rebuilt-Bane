package team3647.frc2026.util;

import java.util.EnumMap;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LEDStates {

    public enum SuperstructureState {
        NONE,
        INTAKING,
        PREPPING,
        SHOOTING;

        public final int priority;

        SuperstructureState() {
            this.priority = ordinal();
        }
    }

    public EnumMap<SuperstructureState, Boolean> states = 
        new EnumMap<SuperstructureState, Boolean>(SuperstructureState.class);

    public double counter = 0;

    public LEDStates() {
        for (SuperstructureState state : SuperstructureState.values()) {
            states.put(state, false);
        }
        states.put(SuperstructureState.NONE, true);
    }

    public SuperstructureState getHighestPriorityState() {
        SuperstructureState highestPriorityState = SuperstructureState.NONE;
        for (SuperstructureState state : SuperstructureState.values()) {
            if (states.get(state) && state.priority > highestPriorityState.priority) {
                highestPriorityState = state;
            }
        }
        return highestPriorityState;
    }

    public void setStateTrue(SuperstructureState newState) {
        states.put(newState, true);
        log();
        counter ++;

    }

    public void setStateFalse(SuperstructureState newState) {
        states.put(newState, false); 
        log();
    }

    public Command setStateTrueCommand(SuperstructureState newState) {
        return Commands.runOnce(() -> setStateTrue(newState));
    }

    public Command setStateFalseCommand(SuperstructureState newState) {
        return Commands.runOnce(() -> setStateFalse(newState));
    }

    /**  
    * use in deadline with a command to set the state true when the command starts and false when it ends 
    */
    public Command activateState(SuperstructureState newState) {
        return Commands.startEnd(() -> setStateTrue(newState), () -> setStateFalse(newState));
    }

    /**
    * use to bind a boolean trigger to a state
    * the state will be true when the trigger is true and false when the trigger is false
    */
    public void bindStateToTrigger(SuperstructureState newState, Trigger trigger) {
        trigger.whileTrue(activateState(newState));
    }

    public void log() {
        for (SuperstructureState state : SuperstructureState.values()) {
            Logger.recordOutput(state.toString(), states.get(state));
        }
        Logger.recordOutput("counter", counter);
    }
}
