package team3647.frc2026.constants;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;

public class GlobalConstants {
    public static final double kDt = 0.02;

    public static final double kFalconTicksPerRotation = 1;

    public static final Mass kRobotMass = Units.Kilogram.of(125);
    public static final MomentOfInertia kRobotMoi = Units.KilogramSquareMeters.of(3.20472599);

    public static final String kDriveCanbusName = "drive";
    public static final String kSubsystemCanbusName = "subsystems";

    public static double kNominalVoltage = 11.0;

    // every single motor id is wrong at this current moment in time

    public class ShooterIds {
        public static final int kShooterOneId = 27;
        public static final int kShooterTwoId = 28;
    }

    public class IndexerIds {
        public static final int kMasterId = 21;
        public static final int kIndexerMasterId = 31; // left indexer
        public static final int kIndexerSlaveId = 27; // right indexer
    }

    public class HoodIds {
        public static final int kMasterId = 29;
    }

    public class WristIds {
        public static final int kMasterId = 32; // right wrist
        public static final int kSlaveId = 30; // left wrist
    }

    public class RollerIds {
        public static final int kMasterId = 36;
    }
}
