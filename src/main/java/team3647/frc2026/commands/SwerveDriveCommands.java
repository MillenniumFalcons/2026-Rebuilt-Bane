package team3647.frc2026.commands;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import team3647.frc2026.subsystems.Swerve.SwerveDrive;
import team3647.frc2026.util.AutoDrive.DriveMode;
import team3647.lib.team9442.AllianceObserver;

public class SwerveDriveCommands implements AllianceObserver {
    private final SwerveDrive swerve;
    private final double kMaxSpeedMps;
    private final double kMaxRotRadPs;

    @SuppressWarnings("unused")
    private Alliance color = Alliance.Red;

    public SwerveDriveCommands(SwerveDrive swerve, double maxSpeedMps, double kMaxRotRadPs) {
        this.swerve = swerve;
        this.kMaxSpeedMps = maxSpeedMps;
        this.kMaxRotRadPs = kMaxRotRadPs;
    }

    @Override
    public void onAllianceFound(Alliance color) {
        this.color = color;
    }

    public Command drive(
            DoubleSupplier x, // X axis on joystick is Left/Right
            DoubleSupplier y, // Y axis on Joystick is Front/Back
            DoubleSupplier rot,
            Supplier<Twist2d> autoDriveVelocities,
            Supplier<DriveMode> getMode,
            BooleanSupplier autoDriveEnabled,
            BooleanSupplier robotRelative) {

        return Commands.run(
                () -> {
                    var isAutoDrive = autoDriveEnabled.getAsBoolean();
                    var velocities = autoDriveVelocities.get();
                    Logger.recordOutput("Velocities", velocities);
                    var robotRel = robotRelative.getAsBoolean();
                    SmartDashboard.putBoolean("MONBEIONS", robotRel);
                    // Logger.recordOutput("hastarget@cmds", toRun);

                    int invert = 1;
                    if (DriverStation.getAlliance().isPresent()) {
                        if (DriverStation.getAlliance().get() == Alliance.Red) {
                            invert = -1;
                        }
                    }

                    double ySquared =
                            Math.pow(y.getAsDouble(), 2) * Math.signum(y.getAsDouble()) * 1.05;
                    double xSquared =
                            Math.pow(x.getAsDouble(), 2) * Math.signum(x.getAsDouble()) * 1.05;

                    double motionXComponent = ySquared * invert * kMaxSpeedMps;
                    double motionYComponent = -xSquared * invert * kMaxSpeedMps;
                    double motionTurnComponent = rot.getAsDouble() * kMaxRotRadPs * 0.6;

                    if (!isAutoDrive) {
                        if (robotRel && Utils.isSimulation()) {
                            swerve.drive(-motionXComponent, -motionYComponent, motionTurnComponent);
                        } else {
                            swerve.driveFieldOriented(
                                    motionXComponent, motionYComponent, motionTurnComponent);
                        }
                    } else if (isAutoDrive) {
                        if (getMode.get().equals(DriveMode.SHOOT_ON_THE_MOVE)) {

                            motionTurnComponent = velocities.dtheta;
                            swerve.driveFieldOriented(
                                    motionXComponent * 0.25,
                                    motionYComponent * 0.25,
                                    motionTurnComponent);
                        }
                    }
                    // else if (isAutoDrive
                    //         && (getMode.get().equals(DriveMode.SCORE)
                    //                 || getMode.get().equals(DriveMode.ALGAE_REEF))) {
                    //     motionXComponent = velocities.dx;
                    //     motionYComponent = velocities.dy;
                    //     motionTurnComponent = velocities.dtheta;

                    //     swerve.drive(motionXComponent, motionYComponent, motionTurnComponent);

                    // } else if (isAutoDrive && getMode.get().equals(DriveMode.BARGE)) {
                    //     motionXComponent = velocities.dx;
                    //     motionYComponent =
                    //             Math.abs(x.getAsDouble()) < 0.05 ? velocities.dy :
                    // -x.getAsDouble();
                    //     motionTurnComponent = velocities.dtheta;
                    //     swerve.drive(motionXComponent, motionYComponent, motionTurnComponent);
                    // } else if (getMode.get().equals(DriveMode.ROBOT_REL)) {
                    //     swerve.drive(motionXComponent, motionYComponent, motionTurnComponent);
                    // }
                },
                swerve);
    }
}
