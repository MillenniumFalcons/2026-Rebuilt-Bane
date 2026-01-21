package team3647.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2026.subsystems.ShooterOne;
import team3647.frc2026.subsystems.ShooterTwo;

public class ShooterCommands {
    ShooterOne shooterOne;
    ShooterTwo shooterTwo;

    public ShooterCommands(ShooterOne shooterOne, ShooterTwo shooterTwo) {
        this.shooterOne = shooterOne;
        this.shooterTwo = shooterTwo;
    }

    public Command setOpenLoop(double output) {
        return Commands.run(
                () -> {
                    shooterOne.setOpenLoop(output);
                    shooterTwo.setOpenLoop(output);
                },
                shooterOne, shooterTwo);
    }

    public Command kill() {
        return setOpenLoop(0).withTimeout(0.3);
    }

    public Command setVelocity(double vel) {
        return Commands.parallel(
            Commands.run(() -> shooterOne.setVelocity(vel, 0), shooterOne),
            Commands.run(() -> shooterTwo.setVelocity(vel, 0), shooterTwo)
        );
    }

    public boolean isAtVelocity(double velocity, double tolerance) {
        return (Math.abs(shooterOne.getVelocity() - velocity) < tolerance) && (Math.abs(shooterTwo.getVelocity() - velocity) < tolerance);
    }

    public Command waitUntilAtVelocity(double velocity, double tolerance) {
        return Commands.waitUntil(() -> isAtVelocity(velocity, tolerance));
    }
}