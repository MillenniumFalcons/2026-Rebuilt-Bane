package team3647.frc2026.constants;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public class FieldConstants {

    public static final Rotation2d kDefaultRot = new Rotation2d();

    public static Distance kFieldLength = Centimeter.of(1755);
    public static double kFieldLengthM = kFieldLength.in(Meter);
    public static Distance kFieldWidth = Centimeter.of(805);
    public static double kFieldWidthM = kFieldWidth.in(Meter);

    public static final Pose2d hubPose =
            new Pose2d(4.622622013092041, 4.039472579956055, new Rotation2d());
    public static final Pose2d feedPose =
            new Pose2d(1.9182995557785034, 3.905377149581909, new Rotation2d());
}
