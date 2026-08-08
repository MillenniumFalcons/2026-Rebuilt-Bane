package team3647.frc2026.util;

import edu.wpi.first.wpilibj.RobotController;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import team3647.frc2026.subsystems.Swerve.SwerveDrive;
import team3647.lib.TalonFXSubsystem;
import team3647.lib.team6328.VirtualSubsystem;

public class CurrentMonitor extends VirtualSubsystem {

    private final SwerveDrive swerve;
    private final List<TalonFXSubsystem> subsystems;
    private final String[] subsystemKeys;

    public CurrentMonitor(SwerveDrive swerve, List<TalonFXSubsystem> subsystems) {
        this.swerve = swerve;
        this.subsystems = subsystems;
        this.subsystemKeys = new String[subsystems.size()];
        for (int i = 0; i < subsystems.size(); i++) {
            subsystemKeys[i] = "CurrentMonitor/" + subsystems.get(i).getName();
        }
    }

    @Override
    public void periodic() {
        double swerveDrive = swerve.getTotalDriveCurrent();
        double swerveSteer = swerve.getTotalSteerCurrent();
        double total = swerveDrive + swerveSteer;

        Logger.recordOutput("CurrentMonitor/Swerve/DriveMotors", swerveDrive);
        Logger.recordOutput("CurrentMonitor/Swerve/SteerMotors", swerveSteer);
        Logger.recordOutput("CurrentMonitor/Swerve/Total", swerveDrive + swerveSteer);

        for (int i = 0; i < subsystems.size(); i++) {
            double current = subsystems.get(i).getTotalCurrent();
            Logger.recordOutput(subsystemKeys[i], current);
            total += current;
        }

        Logger.recordOutput("CurrentMonitor/TotalRobotCurrent", total);
        Logger.recordOutput("CurrentMonitor/BatteryVoltage", RobotController.getBatteryVoltage());
    }
}
