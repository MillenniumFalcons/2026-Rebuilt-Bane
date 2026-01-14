package team3647.frc2026.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoMode {

    public Command autoCommand;

    public Pose2d startingPose;

    public String name;

    public AutoMode(Command autoCommand, Pose2d startingPose, String name) {
        this.autoCommand = autoCommand;
        this.startingPose = startingPose;
        this.name = name;
    }

    public Command getAutoCommand() {
        return autoCommand;
    }

    public Pose2d getStartingPose() {
        return startingPose;
    }

    public String getName() {
        return name;
    }
}
