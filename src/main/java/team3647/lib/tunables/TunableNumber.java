package team3647.lib.tunables;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TunableNumber {
    private final LoggedNetworkNumber number;
    private double lastValue;

    public TunableNumber(String key, double defaultValue) {
        number = new LoggedNetworkNumber(key, defaultValue);
        lastValue = defaultValue;
    }

    public double get() {
        return number.get();
    }

    public boolean hasChanged() {
        double current = number.get();
        if (current != lastValue) {
            lastValue = current;
            return true;
        }
        return false;
    }
}
