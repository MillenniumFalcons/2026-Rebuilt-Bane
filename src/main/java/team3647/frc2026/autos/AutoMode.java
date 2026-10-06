package team3647.frc2026.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class AutoMode {

    public Command autoCommand;

    public Supplier<Pose2d> startingPoseSupplier;

    public String name;

    public AutoMode(Command autoCommand, Supplier<Pose2d> startingPoseSupplier, String name) {
        this.autoCommand = autoCommand;
        this.startingPoseSupplier = startingPoseSupplier;
        this.name = name;
    }

    public AutoMode(Command autoCommand, Pose2d startingPose, String name) {
        this(autoCommand, () -> startingPose, name);
    }

    public Command getAutoCommand() {
        return autoCommand;
    }

    public Pose2d getStartingPose() {
        return startingPoseSupplier.get();
    }

    public String getName() {
        return name;
    }
}
