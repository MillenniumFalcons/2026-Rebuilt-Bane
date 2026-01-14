package team3647.frc2026.constants;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import java.util.List;

public class FieldConstants {

    public static final Rotation2d kDefaultRot = new Rotation2d();

    public static Distance kFieldLength = Centimeter.of(1755);
    public static double kFieldLengthM = kFieldLength.in(Meter);
    public static Distance kFieldWidth = Centimeter.of(805);
    public static double kFieldWidthM = kFieldWidth.in(Meter);
}
