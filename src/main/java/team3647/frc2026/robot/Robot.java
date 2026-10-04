// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2026.robot;

import com.ctre.phoenix6.SignalLogger;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RuntimeType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import team3647.frc2026.constants.VisionConstants;
import team3647.lib.ModifiedSignalLogger;
import team3647.lib.team6328.VirtualSubsystem;
import team3647.lib.vision.LimelightHelpers;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private final boolean isReplay = false;

    public Robot() {
        NetworkTableInstance.getDefault().startServer();
        Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

        if (super.getRuntimeType().equals(RuntimeType.kRoboRIO2)) {
            Logger.addDataReceiver(
                    new WPILOGWriter("/home/lvuser/logs")); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, ModuleType.kRev); // Enables power distribution nlogging
        } else if (super.getRuntimeType().equals(RuntimeType.kRoboRIO)) {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());

        } else {
            setUseTiming(true);
            if (isReplay) {
                String logPath =
                        LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or
                // prompt the
                // user)
                Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
                Logger.addDataReceiver(
                        new WPILOGWriter(
                                LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to
                // a new log
            } else {
                setUseTiming(true);
                // Force a specific folder for Sim logs
                String simLogPath = "sim_logs";
                // Logger.addDataReceiver(new WPILOGWriter(simLogPath));
                Logger.addDataReceiver(new NT4Publisher());
            }
        }

        AutoLogOutputManager.addPackage("team3647.lib");
        AutoLogOutputManager.addPackage("team3647.frc2025.subsystems.Drivetrain");

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata
        // values may
        // be added.
        // SignalLogger.setPath("/home/lvuser/logs");
        // ModifiedSignalLogger.setPath("/home/lvuser/logs");

        LiveWindow.disableAllTelemetry();
        LiveWindow.setEnabled(false);
        SignalLogger.enableAutoLogging(false);

        // SignalLogger.start();
        // ModifiedSignalLogger.start();
        SignalLogger.stop();
        m_robotContainer = new RobotContainer();
        RobotController.setBrownoutVoltage(6.0);
    }

    @Override
    public void robotPeriodic() {
        // Feed odometry into VisionState before CommandScheduler runs VisionSubsystem
        m_robotContainer.updateVisionState();

        CommandScheduler.getInstance().run();

        m_robotContainer.updateRobotPoseForSmartdashboard();
        VirtualSubsystem.periodicAll();
    }

    @Override
    public void disabledInit() {
        m_robotContainer.visionIO.setIMUMode(1);
        SignalLogger.stop();
        ModifiedSignalLogger.stop();
        LimelightHelpers.SetThrottle(VisionConstants.kLimelightATableName, 100);
        LimelightHelpers.SetThrottle(VisionConstants.kLimelightBTableName, 100);
        LimelightHelpers.SetThrottle(VisionConstants.kLimelightCTableName, 100);
    }

    @Override
    public void disabledPeriodic() {
        m_robotContainer.allianceChecker.periodic();
    }

    @Override
    public void disabledExit() {
        LimelightHelpers.cancelThrottle(VisionConstants.kLimelightATableName);
        LimelightHelpers.cancelThrottle(VisionConstants.kLimelightBTableName);
        LimelightHelpers.cancelThrottle(VisionConstants.kLimelightCTableName);
    }

    @Override
    public void autonomousInit() {

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        // ModifiedSignalLogger.start();
        ModifiedSignalLogger.stop();
        m_robotContainer.visionIO.setIMUMode(4);
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
        // comment this out when running real robot
        SimulatedArena.getInstance().simulationPeriodic();
        DogLog.log(
                "Simulation/FuelPoses",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }
}
