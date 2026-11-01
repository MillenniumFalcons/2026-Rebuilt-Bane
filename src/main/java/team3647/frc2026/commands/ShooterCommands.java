package team3647.frc2026.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import team3647.frc2026.subsystems.Shooter;
import team3647.lib.tunables.TunableNumber;

public class ShooterCommands {
    Shooter shooter;
    TunableNumber interpSpeed = new TunableNumber("interp speed", 5.0);

    public ShooterCommands(Shooter shooter) {
        this.shooter = shooter;
    }

    public Command setOpenLoop(double output) {
        return Commands.run(() -> shooter.setOpenLoop(output), shooter);
    }

    public Command kill() {
        return setOpenLoop(0).withTimeout(0.3);
    }

    public Command setVelocity(DoubleSupplier vel) {
        return Commands.run(() -> shooter.setVelocityFOC(vel.getAsDouble()), shooter);
    }

    /**
     * Idle/default command: hold the shooter at {@code idleVel}, but coast (open-loop 0) whenever it
     * is spinning faster than that setpoint so it spins down naturally after a shot instead of being
     * actively braked. While prepping, the setpoint tracks the live shot speed, so keep full
     * velocity control there — it still needs to actively slow down if the robot drives closer.
     */
    public Command idleCoast(DoubleSupplier idleVel) {
        return Commands.run(
                () -> {
                    if (!shooter.isPrepping() && shooter.getVelocity() > idleVel.getAsDouble()) {
                        shooter.setOpenLoop(0);
                    } else {
                        shooter.setVelocityFOC(idleVel.getAsDouble());
                    }
                },
                shooter);
    }

    public Command shootBangBang(DoubleSupplier vel) {
        return Commands.run(() -> shooter.setVelocityFOCBangBang(vel.getAsDouble()), shooter);
    }

    public boolean isAtVelocity(DoubleSupplier velocity) {
        if (velocity.getAsDouble() > 12.0) {
            return Math.abs(shooter.getVelocity() - velocity.getAsDouble()) < 1;
        }
        return shooter.getVelocity() > velocity.getAsDouble() - 0.1;
    }

    public boolean isAtVelocity(DoubleSupplier velocity, double tolerance) {
        return Math.abs(shooter.getVelocity() - velocity.getAsDouble()) < tolerance;
    }

    public Command setVelocityTunable() {
        return Commands.run(() -> shooter.setVelocityFOC(interpSpeed.get()), shooter);
    }
}

//     ShooterLeft shooterOne;
//     ShooterRight shooterTwo;

//     public ShooterCommands(ShooterLeft shooterOne, ShooterRight shooterTwo) {
//         this.shooterOne = shooterOne;
//         this.shooterTwo = shooterTwo;
//     }

//     public Command setOpenLoop(double output) {
//         return Commands.run(
//                 () -> {
//                     shooterOne.setOpenLoop(output);
//                     shooterTwo.setOpenLoop(output);
//                 },
//                 shooterOne,
//                 shooterTwo);
//     }

//     public Command kill() {
//         return setOpenLoop(0).withTimeout(0.3);
//     }

//     public Command killLeft() {
//         return Commands.runOnce(() -> shooterOne.setOpenLoop(0), shooterOne).withTimeout(0.3);
//     }

//     public Command killRight() {
//         return Commands.runOnce(() -> shooterTwo.setOpenLoop(0), shooterTwo).withTimeout(0.3);
//     }

//     public Command setVelocity(DoubleSupplier vel) {
//         return Commands.run(
//                 () -> {
//                     shooterOne.setVelocityVoltage(vel.getAsDouble());
//                     shooterTwo.setVelocityVoltage(vel.getAsDouble());
//                 },
//                 shooterOne,
//                 shooterTwo);
//     }

//     public boolean isAtVelocity(DoubleSupplier velocity, double tolerance) {
//         // Logger.recordOutput("shooter one", Math.abs(shooterOne.getVelocity()));
//         // Logger.recordOutput("shooter two", Math.abs(shooterTwo.getVelocity()));
//         // Logger.recordOutput("veloc", velocity.getAsDouble());
//         return (Math.abs(shooterOne.getVelocity() - velocity.getAsDouble()) < tolerance)
//                 && (Math.abs(shooterTwo.getVelocity() - velocity.getAsDouble()) < tolerance);
//     }

//     public Command waitUntilAtVelocity(DoubleSupplier velocity, double tolerance) {
//         return Commands.waitUntil(() -> isAtVelocity(velocity, tolerance));
//     }
// }
