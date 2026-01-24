package team3647.frc2026.commands;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import team3647.frc2026.constants.WristConstants;
import team3647.frc2026.subsystems.Wrist;

public class WristCommands {
    public Command setAngle(Angle angle) {
        return Commands.run(() -> wrist.setAngle(angle), wrist);
    }

    public Command setAngle(Supplier<Angle> angle) {
        return Commands.run(() -> wrist.setAngle(angle.get()), wrist);  
    }

    public Command setAngleDeg(DoubleSupplier angle) {
        return Commands.run(() -> wrist.setAngleDegs(angle.getAsDouble()), wrist);
    }

    public Command setAngleDeg(double angle) {
        return Commands.run(() -> wrist.setAngleDegs(angle), wrist);
    }

    public Command holdPositionAtCall() {
        return new Command() {
            double degreeAtStart = WristConstants.kInitialAngle;

            @Override
            public void initialize() {
                degreeAtStart = wrist.getAngleDegs();
            }

            @Override
            public void execute() {
                wrist.setAngleDegs(degreeAtStart);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return requirements;
            }
        };
    }

    private final Set<Subsystem> requirements;
    private final Wrist wrist;

    public WristCommands(Wrist wrist) {
        this.wrist = wrist;
        this.requirements = Set.of(wrist);
    }
}
