package team3647.frc2026.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public class FieldConstants {

    public static final Rotation2d kDefaultRot = new Rotation2d();

    public static final Distance kFieldLength = Meters.of(16.541);
    public static final double kFieldLengthM = kFieldLength.in(Meters);
    public static final Distance kFieldWidth = Meters.of(8.0692);
    public static final double kFieldWidthM = kFieldWidth.in(Meters);

    public static final Pose2d hubPose =
            new Pose2d(kFieldLengthM - 4.624, kFieldWidthM - 4.034, new Rotation2d());
    public static final Pose2d feedPose =
            new Pose2d(1.9182995557785034, 3.905377149581909, new Rotation2d());
}
