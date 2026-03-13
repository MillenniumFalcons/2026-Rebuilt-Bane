package team3647.lib.team9442;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.List;
import java.util.function.Consumer;
import team3647.frc2026.autos.AutoCommands;
import team3647.frc2026.autos.AutoMode;

public class AutoChooser extends SendableChooser<AutoMode> implements AllianceObserver {

    public AutoCommands autoCommands;
    Consumer<Pose2d> setStartPose;
    List<AutoMode> autosList;
    Alliance chachedColor = Alliance.Red;
    public static boolean selected = false;

    public AutoChooser(AutoCommands commands, Consumer<Pose2d> setStartPose) {
        super();
        this.autoCommands = commands;
        this.setStartPose = setStartPose;
        autosList = autoCommands.redAutosList;
        setDefaultOption("DEFAULT AUTO CHANE THIS", autosList.get(0));
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
                    // Logger.recordOutput("sellected", getSelected().getName());
                });
    }

    @Override
    public void onAllianceFound(Alliance color) {
        if (color != chachedColor) {
            if (getSelected() != null) {
                setStartPose.accept(getSelected().getStartingPose());
            }
        }
        autosList = color == Alliance.Blue ? autoCommands.blueAutosList : autoCommands.redAutosList;
        addAutos();
        chachedColor = color;
    }

    public void addAutos() {
        for (AutoMode mode : this.autosList) {
            addOption(mode.getName(), mode);
        }
    }
}
