package team3647.frc2026.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;
import team3647.frc2026.subsystems.ShooterLeft;
import team3647.frc2026.subsystems.ShooterRight;
import team3647.lib.LinearRegression;
import team3647.lib.tunables.TunableNumber;

public class ShooterCommands {
    ShooterLeft shooterOne;
    ShooterRight shooterTwo;
    TunableNumber leftVel, rightVel;
    TunableNumber shootSpeed;

    public ShooterCommands(ShooterLeft shooterOne, ShooterRight shooterTwo) {
        this.shooterOne = shooterOne;
        this.shooterTwo = shooterTwo;
        leftVel = new TunableNumber("Left Shooter Tunable", 0.0);
        rightVel = new TunableNumber("Right Shooter Tunable", 0.0);
        shootSpeed = new TunableNumber("shoot speed", 15);
    }

    public Command setOpenLoop(double output) {
        return Commands.run(
                () -> {
                    shooterOne.setOpenLoop(output);
                    shooterTwo.setOpenLoop(output);
                },
                shooterOne,
                shooterTwo);
    }

    public Command kill() {
        return setOpenLoop(0).withTimeout(0.3);
    }

    public Command killLeft() {
        return Commands.runOnce(() -> shooterOne.setOpenLoop(0), shooterOne).withTimeout(0.3);
    }

    public Command killRight() {
        return Commands.runOnce(() -> shooterTwo.setOpenLoop(0), shooterTwo).withTimeout(0.3);
    }

    public Command setVelocity(DoubleSupplier vel) {
        return Commands.run(
                () -> {
                    shooterOne.setVelocityVoltage(vel.getAsDouble());
                    shooterTwo.setVelocityVoltage(vel.getAsDouble());
                },
                shooterOne,
                shooterTwo);
    }

    public Command shoot() {
        return Commands.run(
                () -> {
                    shooterOne.setVelocityVoltage(shootSpeed.get());
                    shooterTwo.setVelocityVoltage(shootSpeed.get());
                },
                shooterOne,
                shooterTwo);
    }

    public Command tunableVelocity() {
        return Commands.run(
                () -> {
                    shooterOne.setVelocityVoltage(leftVel.get());
                    shooterTwo.setVelocityVoltage(rightVel.get());
                },
                shooterOne,
                shooterTwo);
    }

    public boolean isAtVelocity(double velocity, double tolerance) {
        return (Math.abs(shooterOne.getVelocity() - velocity) < tolerance)
                && (Math.abs(shooterTwo.getVelocity() - velocity) < tolerance);
    }

    public Command waitUntilAtVelocity(double velocity, double tolerance) {
        return Commands.waitUntil(() -> isAtVelocity(velocity, tolerance));
    }

    // change to second shooter after getting gains for first one
    public Command characterize() {
        SlewRateLimiter filter = new SlewRateLimiter(0.1);
        Map<Double, Double> voltageVelocityMap = new HashMap<>();
        return Commands.runEnd(
                () -> {
                    double desiredVoltage = filter.calculate(5);
                    shooterOne.setVoltage(desiredVoltage);
                    voltageVelocityMap.put(desiredVoltage, shooterOne.getVelocity());
                },
                () -> {
                    var xArray =
                            voltageVelocityMap.keySet().stream()
                                    .mapToDouble(Double::doubleValue)
                                    .toArray();
                    var yArray =
                            voltageVelocityMap.values().stream()
                                    .mapToDouble(Double::doubleValue)
                                    .toArray();
                    LinearRegression linReg = new LinearRegression(xArray, yArray);
                    // y = ax + b
                    var a = linReg.slope();
                    var b = linReg.intercept();
                    var kS = -b / a;
                    var kV = a;
                    SmartDashboard.putNumber("shooter kS", kS);
                    SmartDashboard.putNumber("shooter kV", kV);
                },
                shooterOne);
    }
}
