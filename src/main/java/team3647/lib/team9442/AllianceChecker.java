package team3647.lib.team9442;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import team3647.lib.utils.Simshit;

public class AllianceChecker {
    private final List<AllianceObserver> observers = new ArrayList<>();
    private Optional<Alliance> alliance = DriverStation.getAlliance();
    private Alliance cachedColor = Alliance.Red;
    private boolean firstRun = true;

    public void registerObserver(AllianceObserver observer) {
        observers.add(observer);
    }

    public void registerObservers(AllianceObserver... addObservers) {
        for (final AllianceObserver o : addObservers) {
            observers.add(o);
        }
    }

    public void periodic() {
        alliance =
                RobotBase.isReal()
                        ? DriverStation.getAlliance()
                        : Simshit.toAlliance(DriverStationSim.getAllianceStationId());
        // DriverStation.reportError("DIME", false);

        alliance.ifPresent(
                color -> {
                    // DriverStation.reportError("ILOVEWHENUCOUNTME" + color.toString(), false);
                    // DriverStation.reportError("Run method? " + (cachedColor != color), false);
                    // DriverStation.reportError("First run? " + firstRun, false);
                    if (cachedColor != color || firstRun) {
                        DriverStation.reportError(
                                "Ran mathod. First run = "
                                        + firstRun
                                        + "colorcheck = "
                                        + (cachedColor != color),
                                false);

                        observers.forEach(observer -> observer.onAllianceFound(color));

                        if (firstRun) firstRun = false;
                    }
                    cachedColor = color;
                });
    }
}
