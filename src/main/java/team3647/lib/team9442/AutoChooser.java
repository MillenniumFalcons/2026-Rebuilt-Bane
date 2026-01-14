package team3647.lib.team9442;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import team3647.frc2026.autos.AutoCommands;
import team3647.frc2026.autos.AutoMode;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

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
        autosList = new ArrayList<AutoMode>();
        onChange(
                (mode) -> {
                    setStartPose.accept(getSelected().startingPose);
                    selected = true;
                    // Logger.recordOutput("sellected", getSelected().getName());
                });
        autosList = autoCommands.redAutosList;
        setDefaultOption("DEFAULT AUTO CHANE THIS", autosList.get(0));
    }

    @Override
    public void onAllianceFound(Alliance color) {
        if (color != chachedColor) {
            setStartPose.accept(getSelected().getStartingPose());
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
