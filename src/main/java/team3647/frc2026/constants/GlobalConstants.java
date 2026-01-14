package team3647.frc2026.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public class GlobalConstants {
    public static final double kDt = 0.02;

    public static final Mass kRobotMass = Units.Kilogram.of(125);
    public static final MomentOfInertia kRobotMoi = Units.KilogramSquareMeters.of(3.20472599);

    public static final String kDriveCanbusName = "rio";
    public static final String kSubsystemCanbusName = "rio";

    public static double kNominalVoltage = 11.0;

    public class RollersIds {
        public static final int kMasterId = 27;
    }

    public class KickerIds {
        public static final int kMasterId = 21;
    }
}
