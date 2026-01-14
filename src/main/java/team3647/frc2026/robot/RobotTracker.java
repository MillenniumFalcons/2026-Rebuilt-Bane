package team3647.frc2026.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

import team3647.frc2026.Util.AutoDrive;
import team3647.frc2026.subsystems.Superstructure;
import team3647.lib.team6328.VirtualSubsystem;

public class RobotTracker extends VirtualSubsystem {
    private final Superstructure superstructure;
    private final AutoDrive autoDrive;

    Translation2d fk = new Translation2d();

    public RobotTracker(Superstructure superstructure, AutoDrive autoDrive) {
        super();
        this.superstructure = superstructure;
        this.autoDrive = autoDrive;
    }

        @Override
        public void periodic() {
                // TODO Auto-generated method stub
                throw new UnsupportedOperationException("Unimplemented method 'periodic'");
        }
}
