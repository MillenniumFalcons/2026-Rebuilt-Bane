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
    }

    public class IndexerIds {
        public static final int kIndexerMasterId = 16;
    }

    public class KickerIds {
        public static final int kMasterId = 17;
        public static final int kSlave1Id = 18;
        public static final int kSlave2Id = 19;
        public static final int kSlave3Id = 20;
    }

    public class ShooterIds {
        public static final int kMasterId = 21;
        public static final int kSlaveId = 22;
    }

    public class HoodIds {
        public static final int kMasterId = 23;
        public static final int kCanCoderId = 24;
    }
}

// BETA CONSTANTS
// public class GlobalConstants {
//     public static final double kDt = 0.02;

//     public static final double kFalconTicksPerRotation = 1;

//     public static final Mass kRobotMass = Units.Kilogram.of(125); // WRONG
//     public static final MomentOfInertia kRobotMoi =
//             Units.KilogramSquareMeters.of(3.20472599); // WRONG

//     public static final String kDriveCanbusName = "drive";
//     public static final String kSubsystemCanbusName = "subsystems";

//     public static double kNominalVoltage = 11.0;

//     public class WristIds {
//         public static final int kMasterId = 14;
//     }

//     public class RollerIds {
//         public static final int kMasterId = 15;
//     }

//     public class IndexerIds {
//         public static final int kIndexerMasterId = 16; // left indexer
//         public static final int kIndexerSlaveId = 17; // right indexer
//     }

//     public class KickerIds {
//         public static final int kMasterId = 18;
//     }

//     public class ShooterIds {
//         public static final int kShooterOneId = 19; // left
//         public static final int kShooterOneSlaveId = 20; // left slave
//         public static final int kShooterTwoId = 21; // right
//         public static final int kShooterTwoSlaveId = 22; // right slave
//     }

//     public class HoodIds {
//         public static final int kMasterId = 23;
//         public static final int kCanCoderId = 26;
//     }

//     public class ClimbIds {
//         public static final int kLeftId = 24;
//         public static final int kRightId = 25;
//     }
// }
