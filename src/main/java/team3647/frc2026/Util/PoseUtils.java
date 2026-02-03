package team3647.frc2026.Util;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;

public class PoseUtils {

    public static boolean inCircle(Pose2d pose, Pose2d center, double radiusM) {
        return Math.abs(pose.getTranslation().getDistance(center.getTranslation())) <= radiusM;
    }

    public static boolean aligned(
            Pose2d pose, Pose2d center, double radiusM, double rotThresholdDeg) {
        return Math.abs(pose.getTranslation().getDistance(center.getTranslation())) <= radiusM
                && Math.abs(pose.getRotation().minus(center.getRotation()).getDegrees())
                        <= rotThresholdDeg;
    }

    public static boolean inCircle(Pose2d pose, Pose2d center, Distance radius) {
        return inCircle(pose, center, radius.in(Meter));
    }

    public static boolean inRect(
            Translation2d point, double lowX, double highX, double lowY, double highY) {
        return point.getX() > lowX
                && point.getX() < highX
                && point.getY() > lowY
                && point.getY() < highY;
    }

    public static boolean inRect(Translation2d point, Rectangle2d rect) {
        return rect.contains(point);
    }

    public static boolean inRect(Pose2d pose, Rectangle2d rect) {
        return inRect(pose.getTranslation(), rect);
    }
}
