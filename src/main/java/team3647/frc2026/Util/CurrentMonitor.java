package team3647.frc2026.util;

import java.util.List;
import org.littletonrobotics.junction.Logger;
import team3647.frc2026.subsystems.Swerve.SwerveDrive;
import team3647.lib.TalonFXSubsystem;
import team3647.lib.team6328.VirtualSubsystem;

public class CurrentMonitor extends VirtualSubsystem {

    private final SwerveDrive swerve;
    private final List<TalonFXSubsystem> subsystems;

    public CurrentMonitor(SwerveDrive swerve, List<TalonFXSubsystem> subsystems) {
        this.swerve = swerve;
        this.subsystems = subsystems;
    }

    @Override
    public void periodic() {
        double swerveDrive = swerve.getTotalDriveCurrent();
        double swerveSteer = swerve.getTotalSteerCurrent();
        double total = swerveDrive + swerveSteer;

        Logger.recordOutput("CurrentMonitor/Swerve/DriveMotors", swerveDrive);
        Logger.recordOutput("CurrentMonitor/Swerve/SteerMotors", swerveSteer);
        Logger.recordOutput("CurrentMonitor/Swerve/Total", swerveDrive + swerveSteer);

        for (TalonFXSubsystem subsystem : subsystems) {
            double current = subsystem.getTotalCurrent();
            Logger.recordOutput("CurrentMonitor/" + subsystem.getName(), current);
            total += current;
        }

        Logger.recordOutput("CurrentMonitor/TotalRobotCurrent", total);
    }
}
