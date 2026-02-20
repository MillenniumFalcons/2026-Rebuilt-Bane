package team3647.frc2026.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;

public class FieldConstants {

    public static final Rotation2d kDefaultRot = new Rotation2d();

    public static Distance kFieldLength = Inches.of(651.22);
    public static double kFieldLengthM = kFieldLength.in(Meter);
    public static Distance kFieldWidth = Inches.of(317.69);
    public static double kFieldWidthM = kFieldWidth.in(Meter);

    // wait i can't remember what it is lmao
    // 16.54 -
    // 8.069 -
    public static final Pose2d hubPose =
            new Pose2d(16.54 - 4.624, 8.069 - 4.034, new Rotation2d());
    public static final Pose2d feedPose =
            new Pose2d(1.9182995557785034, 3.905377149581909, new Rotation2d());
}
