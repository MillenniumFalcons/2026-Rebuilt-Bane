package team3647.lib.tunables;

import edu.wpi.first.math.controller.PIDController;
import team3647.lib.team6328.VirtualSubsystem;

public class TunablePID extends VirtualSubsystem {

    public TunableNumber kP, kI, kD;
    private PIDController controller;

    public TunablePID(String name, PIDController controller) {
        this.controller = controller;
        kP = new TunableNumber(name + "/kP", controller.getP());
        kI = new TunableNumber(name + "/kI", controller.getI());
        kD = new TunableNumber(name + "/kD", controller.getD());
    }

    public void update() {
        if (hasChanged()) {
            controller.setP(kP.get());
            controller.setI(kI.get());
            controller.setD(kD.get());
        }
    }

    private boolean hasChanged() {
        return kP.hasChanged() || kI.hasChanged() || kD.hasChanged();
    }

    @Override
    public void periodic() {
        update();
    }
}
