package team3647.frc2026.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;
import team3647.lib.team6328.VirtualSubsystem;

public class MatchStateLogger extends VirtualSubsystem {

    @Override
    public void periodic() {
        double matchTime = Timer.getMatchTime();
        Logger.recordOutput("MatchState/TimeRemaining", matchTime);

        if (matchTime < 0) {
            Logger.recordOutput("MatchState/Period", "Disabled");
            Logger.recordOutput("MatchState/Shift", 0);
            Logger.recordOutput("MatchState/ShiftTimeRemaining", 0.0);
            Logger.recordOutput("MatchState/HubActive", false);
            return;
        }

        if (DriverStation.isAutonomous()) {
            Logger.recordOutput("MatchState/Period", "Auto");
            Logger.recordOutput("MatchState/Shift", 0);
            Logger.recordOutput("MatchState/ShiftTimeRemaining", 0.0);
            Logger.recordOutput("MatchState/HubActive", true);
            return;
        }

        String period;
        int shift;
        double shiftTimeRemaining;

        if (matchTime > 130) {
            period = "TRANSITION";
            shift = 0;
            shiftTimeRemaining = matchTime - 130;
        } else if (matchTime > 105) {
            period = "SHIFT 1";
            shift = 1;
            shiftTimeRemaining = matchTime - 105;
        } else if (matchTime > 80) {
            period = "SHIFT 2";
            shift = 2;
            shiftTimeRemaining = matchTime - 80;
        } else if (matchTime > 55) {
            period = "SHIFT 3";
            shift = 3;
            shiftTimeRemaining = matchTime - 55;
        } else if (matchTime > 30) {
            period = "SHIFT 4";
            shift = 4;
            shiftTimeRemaining = matchTime - 30;
        } else {
            period = "ENDGAME";
            shift = 0;
            shiftTimeRemaining = 0;
        }

        Logger.recordOutput("MatchState/Period", period);
        Logger.recordOutput("MatchState/Shift", shift);
        Logger.recordOutput("MatchState/ShiftTimeRemaining", shiftTimeRemaining);
        Logger.recordOutput("MatchState/HubActive", isHubActive(matchTime));
        Logger.recordOutput("MatchState/WonAuto", isWonAuto());
    }

    public boolean isWonAuto() {
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData == null || gameData.isEmpty()) return true;

        DriverStation.Alliance ourAlliance =
                DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        return ourAlliance == DriverStation.Alliance.Red
                ? gameData.equals("B") // Blue lost → Red won
                : gameData.equals("R"); // Red lost → Blue won
    }

    // Returns true when our alliance's goal is active (can score). Uses FMS game data
    // to determine auto winner, then checks shift timing. Always true in transition and endgame.
    public boolean isHubActive(double matchTime) {
        if (DriverStation.isAutonomous() || matchTime > 130 || matchTime <= 30) return true;

        // Odd shifts (1, 3): 105-130s and 55-80s remaining
        boolean isOddShift = matchTime > 105 || (matchTime > 55 && matchTime <= 80);

        boolean wonAuto = isWonAuto();

        return wonAuto ? isOddShift : !isOddShift;
    }
}
