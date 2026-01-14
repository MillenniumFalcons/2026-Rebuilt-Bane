package team3647.lib.utils;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;

public class GeomUtil {
    public static double distance(Pose2d pose1, Pose2d pose2) {
        final var pose = pose1.minus(pose2);
        return Math.sqrt(Math.pow(pose.getX(), 2) + Math.pow(pose.getY(), 2));
    }

    public static double distanceSquared(Pose2d pose1, Pose2d pose2) {
        final var pose = pose1.minus(pose2);
        return Math.pow(pose.getX(), 2) + Math.pow(pose.getY(), 2);
    }

    public static double distance(Transform2d transform) {
        var distancSquared = Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2);
        var distance = Math.sqrt(distancSquared);
        return distance;
    }

    public static double distanceSquared(Transform2d transform) {
        var distancSquared = Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2);
        return distancSquared;
    }

    public static double distanceSquared(Translation2d transform) {
        var distancSquared = Math.pow(transform.getX(), 2) + Math.pow(transform.getY(), 2);
        return distancSquared;
    }

    public static Vector<N3> asVector(ChassisSpeeds speeds) {
        return VecBuilder.fill(
                speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

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

    /**
     * Checks if a point is in a triangle, inclusive of edges
     *
     * @param pose The point
     * @param a The first point of the triangle
     * @param b The second point of the triangle
     * @param c The third point of the triangle
     * @return If the point is in the triangle
     *     <p>Explanation: https://youtu.be/yyJ-hdISgnw?si=2cvLFURBAWFPHvug&t=158
     */
    public static boolean inTriangle(Pose2d pose, Pose2d a, Pose2d b, Pose2d c) {

        final Vector<N2> abPerp = VecBuilder.fill(-(b.getY() - a.getY()), b.getX() - a.getX());
        final Vector<N2> bcPerp = VecBuilder.fill(-(c.getY() - b.getY()), c.getX() - b.getX());
        final Vector<N2> caPerp = VecBuilder.fill(-(a.getY() - c.getY()), a.getX() - c.getX());

        final Vector<N2> ap = VecBuilder.fill(pose.getX() - a.getX(), (pose.getY() - a.getY()));
        final Vector<N2> bp = VecBuilder.fill(pose.getX() - b.getX(), (pose.getY() - b.getY()));
        final Vector<N2> cp = VecBuilder.fill(pose.getX() - c.getX(), (pose.getY() - c.getY()));

        double aSign = Math.signum(abPerp.dot(ap) == 0 ? 1 : abPerp.dot(ap));
        double bSign = Math.signum(bcPerp.dot(bp) == 0 ? 1 : bcPerp.dot(bp));
        double cSign = Math.signum(caPerp.dot(cp) == 0 ? 1 : caPerp.dot(cp));

        return aSign == bSign && bSign == cSign;
    }
}
