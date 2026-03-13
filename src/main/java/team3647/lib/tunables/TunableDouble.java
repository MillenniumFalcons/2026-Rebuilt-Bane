package team3647.lib.tunables;

import edu.wpi.first.math.controller.PIDController;
import team3647.lib.team6328.VirtualSubsystem;

public class TunableDouble extends VirtualSubsystem {

    public TunableNumber tunableNumber;
    public Double doubleNumber;

    public TunableDouble(String name, Double doubleNumber) {
        this.doubleNumber = doubleNumber;
        tunableNumber = new TunableNumber(name, doubleNumber);
    }

    public void update() {
        if (hasChanged()) {
            doubleNumber = tunableNumber.get();
        }
    }

    private boolean hasChanged() {
        return tunableNumber.hasChanged();
    }

    @Override
    public void periodic() {
        update();
    }
}
