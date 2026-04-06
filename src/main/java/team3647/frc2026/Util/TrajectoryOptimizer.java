package team3647.frc2026.util;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Standalone trajectory optimizer that matches MapleSim physics (g=11 m/s², parabolic). Finds the
 * most robust (exit speed, hood angle) pair at each distance for scoring into the hub.
 *
 * <p>Strategy: for each distance, map the full 2D scoring region in (speed, angle) space. Then find
 * the centroid — this naturally picks a point in the middle of the region, balancing between flat
 * (fast, low angle, short ToF) and lob (slow, high angle, long ToF) trajectories. The centroid
 * maximizes margin in all directions.
 *
 * <p>Run: java -cp build/classes/java/main team3647.frc2026.util.TrajectoryOptimizer
 */
public class TrajectoryOptimizer {

    // Physics
    static final double G = 9.81; // m/s^2 (real gravity)

    // Shooter geometry
    static final double ROLLER_CENTER_HEIGHT = 0.5302; // 20.875 inches in meters
    static final double HOOD_RADIUS = 0.1943; // 7.651 inches in meters

    // Hub target
    static final double HUB_HEIGHT = 1.8288; // 6 feet in meters (lip of scoring funnel)
    static final double HUB_SCORING_RADIUS = 0.5969; // meters (spherical scoring zone)

    // Simulation parameters
    static final double DT = 0.005; // seconds
    static final double MAX_TIME = 3.0; // seconds max flight

    // Search grid (hood angle: angle of radius from horizontal, launch angle = 90 - hood angle)
    static final double HOOD_ANGLE_MIN = 1.0;
    static final double HOOD_ANGLE_MAX = 46.0;
    static final double HOOD_ANGLE_STEP = 0.2;
    static final double SPEED_MIN = 3.0;
    static final double SPEED_MAX = 20.0;
    static final double SPEED_STEP = 0.1;
    static final double DIST_MIN = 1.0;
    static final double DIST_MAX = 8.0;
    static final double DIST_STEP = 0.5;

    // Monte Carlo
    static final int MC_TRIALS = 1000;
    static final double MC_ANGLE_SIGMA = 1.0; // degrees
    static final double MC_SPEED_SIGMA = 0.5; // m/s

    /** Exit height based on hood angle (angle of radius from horizontal). */
    static double exitHeight(double hoodAngleDeg) {
        return ROLLER_CENTER_HEIGHT + HOOD_RADIUS * Math.sin(Math.toRadians(hoodAngleDeg));
    }

    /** Launch angle from horizontal. Ball exits perpendicular to the hood radius. */
    static double launchAngle(double hoodAngleDeg) {
        return 90.0 - hoodAngleDeg;
    }

    /** Returns time of flight if the trajectory scores, -1 otherwise. */
    static double simulateToF(double exitSpeed, double hoodAngleDeg, double targetDist) {
        double launchRad = Math.toRadians(launchAngle(hoodAngleDeg));
        double h0 = exitHeight(hoodAngleDeg);
        double vx = exitSpeed * Math.cos(launchRad);
        double vz = exitSpeed * Math.sin(launchRad);

        if (vx <= 0) return -1;

        int steps = (int) (MAX_TIME / DT);
        for (int i = 1; i <= steps; i++) {
            double t = i * DT;
            double x = vx * t;
            double z = h0 + vz * t - 0.5 * G * t * t;

            double dHoriz = x - targetDist;
            double dVert = z - HUB_HEIGHT;
            double dist = Math.sqrt(dHoriz * dHoriz + dVert * dVert);

            if (dist < HUB_SCORING_RADIUS) {
                return t;
            }

            if (z < 0 && t > 0.05) {
                return -1;
            }
        }
        return -1;
    }

    static boolean scores(double exitSpeed, double angleDeg, double targetDist) {
        return simulateToF(exitSpeed, angleDeg, targetDist) > 0;
    }

    /** Compute entry angle (degrees below horizontal, positive = descending). */
    static double entryAngle(double exitSpeed, double hoodAngleDeg, double targetDist) {
        double launchRad = Math.toRadians(launchAngle(hoodAngleDeg));
        double vx = exitSpeed * Math.cos(launchRad);
        double vz = exitSpeed * Math.sin(launchRad);

        double tof = simulateToF(exitSpeed, hoodAngleDeg, targetDist);
        if (tof < 0) return 0;

        double vzAtHub = vz - G * tof;
        return Math.toDegrees(Math.atan2(-vzAtHub, vx));
    }

    /** Monte Carlo perturbation test. Returns fraction of trials that score (0.0-1.0). */
    static double perturbationTest(double speed, double angle, double distance) {
        Random rng = new Random(42);
        int hits = 0;
        for (int i = 0; i < MC_TRIALS; i++) {
            double pertAngle = angle + rng.nextGaussian() * MC_ANGLE_SIGMA;
            double pertSpeed = speed + rng.nextGaussian() * MC_SPEED_SIGMA;
            if (pertSpeed > 0 && scores(pertSpeed, pertAngle, distance)) {
                hits++;
            }
        }
        return (double) hits / MC_TRIALS;
    }

    /** Find the contiguous hood angle range at a given speed. Returns {min, max} of widest band. */
    static double[] findAngleRange(double speed, double distance) {
        double bestMin = -1, bestMax = -1, bestWidth = 0;
        double curMin = -1;
        boolean inBand = false;

        for (double a = HOOD_ANGLE_MIN; a <= HOOD_ANGLE_MAX + 0.001; a += HOOD_ANGLE_STEP) {
            if (scores(speed, a, distance)) {
                if (!inBand) {
                    curMin = a;
                    inBand = true;
                }
            } else {
                if (inBand) {
                    double width = (a - HOOD_ANGLE_STEP) - curMin;
                    if (width > bestWidth) {
                        bestWidth = width;
                        bestMin = curMin;
                        bestMax = a - HOOD_ANGLE_STEP;
                    }
                    inBand = false;
                }
            }
        }
        if (inBand) {
            double width = HOOD_ANGLE_MAX - curMin;
            if (width > bestWidth) {
                bestMin = curMin;
                bestMax = HOOD_ANGLE_MAX;
            }
        }
        if (bestMin < 0) return null;
        return new double[] {bestMin, bestMax};
    }

    static class Result {
        double distance;
        double angle;
        double exitSpeed;
        double tof;
        double angleMargin;
        double perturbationPct;
        double entryAngle;
        int scoringAreaCells;
    }

    public static void main(String[] args) {
        List<Result> results = new ArrayList<>();

        System.out.println("Running trajectory optimization...");
        System.out.println("Strategy: centroid of 2D scoring region (speed x hood angle)");
        System.out.println("Hood angle = radius from horizontal, launch angle = 90 - hood angle");
        System.out.println("Using g = " + G + " m/s^2\n");

        for (double dist = DIST_MIN; dist <= DIST_MAX + 0.01; dist += DIST_STEP) {

            // Map full scoring region and compute centroid
            double sumAngle = 0, sumSpeed = 0;
            int count = 0;

            for (double speed = SPEED_MIN; speed <= SPEED_MAX; speed += SPEED_STEP) {
                for (double hoodAngle = HOOD_ANGLE_MIN;
                        hoodAngle <= HOOD_ANGLE_MAX;
                        hoodAngle += HOOD_ANGLE_STEP) {
                    if (scores(speed, hoodAngle, dist)) {
                        sumAngle += hoodAngle;
                        sumSpeed += speed;
                        count++;
                    }
                }
            }

            if (count == 0) {
                System.out.printf("  d=%4.1fm  NO VALID TRAJECTORY FOUND\n", dist);
                continue;
            }

            double centroidAngle = sumAngle / count;
            double centroidSpeed = sumSpeed / count;

            // Snap to nearest grid point that actually scores
            // (the centroid itself might not score if the region is concave)
            double bestAngle = centroidAngle;
            double bestSpeed = centroidSpeed;
            if (!scores(bestSpeed, bestAngle, dist)) {
                double minDist = Double.MAX_VALUE;
                for (double speed = SPEED_MIN; speed <= SPEED_MAX; speed += SPEED_STEP) {
                    for (double hoodAngle = HOOD_ANGLE_MIN;
                            hoodAngle <= HOOD_ANGLE_MAX;
                            hoodAngle += HOOD_ANGLE_STEP) {
                        if (scores(speed, hoodAngle, dist)) {
                            double da = hoodAngle - centroidAngle;
                            double ds = speed - centroidSpeed;
                            double d2 = da * da + ds * ds;
                            if (d2 < minDist) {
                                minDist = d2;
                                bestAngle = hoodAngle;
                                bestSpeed = speed;
                            }
                        }
                    }
                }
            }

            double tof = simulateToF(bestSpeed, bestAngle, dist);
            double entry = entryAngle(bestSpeed, bestAngle, dist);
            double[] angleRange = findAngleRange(bestSpeed, dist);
            double angleMargin = (angleRange != null) ? (angleRange[1] - angleRange[0]) : 0;

            Result r = new Result();
            r.distance = dist;
            r.angle = bestAngle;
            r.exitSpeed = bestSpeed;
            r.tof = tof;
            r.angleMargin = angleMargin;
            r.entryAngle = entry;
            r.scoringAreaCells = count;
            r.perturbationPct = perturbationTest(bestSpeed, bestAngle, dist) * 100;
            results.add(r);

            System.out.printf(
                    "  d=%4.1fm  hood=%5.1f°  launch=%5.1f°  speed=%5.2f m/s  ToF=%.3fs"
                            + "  margin=%4.1f°  entry=%4.1f°  area=%5d  MC=%.1f%%\n",
                    r.distance,
                    r.angle,
                    launchAngle(r.angle),
                    r.exitSpeed,
                    r.tof,
                    r.angleMargin,
                    r.entryAngle,
                    r.scoringAreaCells,
                    r.perturbationPct);
        }

        // Results table
        System.out.println("\n=== RESULTS TABLE ===");
        System.out.printf(
                "%-9s| %-11s| %-11s| %-7s| %-7s| %-12s| %-6s| %-6s\n",
                "Distance",
                "Hood Angle",
                "Exit Speed",
                "ToF",
                "Margin",
                "Entry Angle",
                "Area",
                "MC");
        System.out.println(
                "---------|------------|-----------|-------|-------|------------|------|------");
        for (Result r : results) {
            System.out.printf(
                    " %4.1f m  | %5.1f deg  | %5.2f m/s | %.3f | %4.1f° |   %5.1f°    |%5d"
                            + " | %4.1f%%\n",
                    r.distance,
                    r.angle,
                    r.exitSpeed,
                    r.tof,
                    r.angleMargin,
                    r.entryAngle,
                    r.scoringAreaCells,
                    r.perturbationPct);
        }

        // Copy-paste sections
        System.out.println("\n=== COPY-PASTE: HoodConstants.hoodAngleMap ===");
        for (Result r : results) {
            System.out.printf("hoodAngleMap.put(%.1f, %.1f);\n", r.distance, r.angle);
        }

        System.out.println("\n=== COPY-PASTE: ShooterConstants.shootSpeedMap (EXIT SPEED) ===");
        System.out.println("// NOTE: these are EXIT speeds, not surface speeds.");
        System.out.println("// Divide by your ball-to-wheel speed ratio to get surface speed.");
        for (Result r : results) {
            System.out.printf("shootSpeedMap.put(%.1f, %.2f);\n", r.distance, r.exitSpeed);
        }

        System.out.println("\n=== COPY-PASTE: RobotTracker.timeOfFlightMap ===");
        for (Result r : results) {
            System.out.printf("timeOfFlightMap.put(%.1f, %.3f);\n", r.distance, r.tof);
        }
    }
}
