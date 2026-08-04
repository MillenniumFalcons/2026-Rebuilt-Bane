package team3647.frc2026.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public class GlobalConstants {
    public static final double kDt = 0.02;

    public static final double kFalconTicksPerRotation = 1;

    public static final Mass kRobotMass = Units.Kilogram.of(125); // WRONG
    public static final MomentOfInertia kRobotMoi =
            Units.KilogramSquareMeters.of(3.20472599); // WRONG

    public static final String kDriveCanbusName = "drive";
    public static final String kSubsystemCanbusName = "subsystems";

    public static double kNominalVoltage = 11.0;

    public class WristIds {
        public static final int kMasterId = 14;
    }

    public class RollerIds {
        public static final int kMasterId = 15;
        public static final int kSlaveId = 16;
    }

    public class IndexerIds {
        public static final int kIndexerMasterId = 17;
    }

    public class KickerIds {
        public static final int kMasterId = 18;
        public static final int kSlave2Id = 20;
        public static final int kSlave3Id = 21;
    }

    public class ShooterIds {
        public static final int kMasterId = 22;
        public static final int kSlaveId = 23;
        public static final int kSlave1Id = 19;
    }

    public class HoodIds {
        public static final int kMasterId = 24;
    }
}