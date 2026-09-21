package team3647.frc2026.commands;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import team3647.frc2026.subsystems.Swerve.SwerveDrive;
import team3647.frc2026.util.AutoDrive;
import team3647.frc2026.util.AutoDrive.DriveMode;
import team3647.lib.team9442.AllianceObserver;

public class SwerveDriveCommands implements AllianceObserver {
    private final SwerveDrive swerve;
    private final double kMaxSpeedMps;
    private final double kMaxRotRadPs;
    private double rotMultiplier = 0.6;

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
            BooleanSupplier isAligned) {

        return Commands.run(
                () -> {
                    var isAutoDrive = autoDriveEnabled.getAsBoolean();
                    DriveMode mode = getMode.get();
                    var velocities = autoDriveVelocities.get();
                    Logger.recordOutput("autoDriveVelocities", velocities);
                    // SmartDashboard.putBoolean("MONBEIONS", robotRel);
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

                    double xyMult = 1.0;
                    double motionTurnComponent = rot.getAsDouble() * kMaxRotRadPs * rotMultiplier;

                    if (isAutoDrive) {
                        motionTurnComponent = velocities.dtheta;
                        switch (mode) {
                            case SHOOT_ON_THE_MOVE:
                                xyMult = 0.25;
                            case SHOOT:
                                xyMult = 0.25;
                            default:
                                xyMult = 1.0;
                        }
                    }

                    double motionXComponent = xyMult * ySquared * invert * kMaxSpeedMps;
                    double motionYComponent = xyMult * -xSquared * invert * kMaxSpeedMps;
                    // swerve.generate(motionXComponent, motionYComponent, motionTurnComponent);
                    swerve.driveFieldOriented(motionXComponent, motionYComponent, motionTurnComponent);
                },
                swerve);
    }

    public void setRotMultiplier(double multiplier) {
        this.rotMultiplier = multiplier;
    }

    public Command driveButInAuto(AutoDrive autoDrive, Supplier<Twist2d> autoDriveVelocities) {
        return Commands.sequence(
                        Commands.runOnce(() -> swerve.drive(0, 0, 0)),
                        autoDrive.enableAutoDrive(),
                        Commands.run(
                                () ->
                                        swerve.driveFieldOriented(
                                                0, 0, autoDriveVelocities.get().dtheta),
                                swerve))
                .finallyDo(() -> autoDrive.setEnabled(false));
    }
}
