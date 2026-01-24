package team3647.lib.tunables;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import team3647.lib.team6328.VirtualSubsystem;

public class TunableProfiledPID extends VirtualSubsystem {

    public TunableNumber kP, kI, kD, maxVel, maxAccel;
    private ProfiledPIDController controller;

    public TunableProfiledPID(String name, ProfiledPIDController controller) {
        this.controller = controller;
        kP = new TunableNumber(name + "/kP", controller.getP());
        kI = new TunableNumber(name + "/kI", controller.getI());
        kD = new TunableNumber(name + "/kD", controller.getD());
        maxVel = new TunableNumber(name + "/MaxVelocity", controller.getConstraints().maxVelocity);
        maxAccel =
                new TunableNumber(
                        name + "/MaxAcceleration", controller.getConstraints().maxAcceleration);
    }

    public void update() {
        if (hasChanged()) {
            controller.setP(kP.get());
            controller.setI(kI.get());
            controller.setD(kD.get());
            controller.setConstraints(
                    new TrapezoidProfile.Constraints(maxVel.get(), maxAccel.get()));
        }
    }

    private boolean hasChanged() {
        return kP.hasChanged()
                || kI.hasChanged()
                || kD.hasChanged()
                || maxVel.hasChanged()
                || maxAccel.hasChanged();
    }

    @Override
    public void periodic() {
        update();
    }
}
