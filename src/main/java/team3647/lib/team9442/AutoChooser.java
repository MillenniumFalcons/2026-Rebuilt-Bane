package team3647.lib.team9442;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.function.Consumer;
import team3647.frc2026.autos.AutoCommands;
import team3647.frc2026.autos.AutoMode;

public class AutoChooser extends SendableChooser<AutoMode> implements AllianceObserver {

    public AutoCommands autoCommands;
    Consumer<Pose2d> setStartPose;
    public static boolean selected = false;

    public AutoChooser(AutoCommands commands, Consumer<Pose2d> setStartPose) {
        super();
        this.autoCommands = commands;
        this.setStartPose = setStartPose;
        setDefaultOption(commands.autosList.get(0).getName(), commands.autosList.get(0));
        for (int i = 1; i < commands.autosList.size(); i++) {
            AutoMode mode = commands.autosList.get(i);
            addOption(mode.getName(), mode);
        }
        onChange(
                (mode) -> {
                    AutoMode selectedMode = getSelected();
                    Pose2d startingPose;
                    if (selectedMode == null) {
                        startingPose = new Pose2d();
                    } else {
                        startingPose = selectedMode.getStartingPose();
                    }
                    setStartPose.accept(startingPose);
                    selected = true;
                });
    }

    @Override
    public void onAllianceFound(Alliance color) {
        if (getSelected() != null) {
            setStartPose.accept(getSelected().getStartingPose());
        }
    }
}
