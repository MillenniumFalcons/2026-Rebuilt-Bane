package team3647.frc2026.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import team3647.frc2026.constants.HoodConstants;
import team3647.frc2026.subsystems.Hood;

public class HoodCommands {

    public Command setAngle(Angle angle) {
        return Commands.run(() -> hood.setAngle(angle), hood);
    }

    public Command setAngle(Supplier<Angle> angle) {
        return Commands.run(() -> hood.setAngle(angle.get()), hood);
    }

    public Command setAngleDeg(DoubleSupplier angle) {
        return Commands.run(() -> hood.setAngleDegs(angle.getAsDouble()), hood);
    }

    public Command setAngleDeg(double angle) {
        return Commands.run(() -> hood.setAngleDegs(angle), hood);
    }

    public boolean angleReached(DoubleSupplier angle, double tolerance) {
        return Math.abs(hood.getAngleDegs() - angle.getAsDouble()) < tolerance;
    }

    public Command holdPositionAtCall() {
        return new Command() {
            Angle degreeAtStart = HoodConstants.kInitialAngle;

            @Override
            public void initialize() {
                degreeAtStart = hood.getAngle();
            }

            @Override
            public void execute() {
                hood.setAngle(degreeAtStart);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return requirements;
            }
        };
    }

    private final Set<Subsystem> requirements;
    private final Hood hood;

    public HoodCommands(Hood hood) {
        this.hood = hood;
        this.requirements = Set.of(hood);
    }
}
