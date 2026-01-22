package team3647.frc2026.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2026.constants.HoodConstants;

/**
 * Targeting Utility for 2026 FRC Game. Focuses on shooting into a Central Hub with constant
 * flywheel speed but variable hood angle and robot heading. * Uses RobotTracker for physics
 * compensation (shooting on the move).
 */
public class TargetingUtil {

    private final RobotTracker robotTracker;

    // Manual offset for the hood angle (in degrees), adjustable by driver
    private double offset = 0.0;

    // Constant flywheel speed (Matches the 15 m/s or similar logic in RobotTracker)
    private static final double kShootSpeed = 15.0;

    // =========================================================================
    // Lookup Maps
    // =========================================================================
    private final InterpolatingDoubleTreeMap hoodAngleMap = HoodConstants.hoodAngleMap;

    public TargetingUtil(RobotTracker robotTracker) {
        this.robotTracker = robotTracker;
    }

    // =========================================================================
    // Offset Commands
    // =========================================================================

    public Command offsetUp() {
        return Commands.runOnce(() -> offset += 0.5);
    }

    public Command offsetDown() {
        return Commands.runOnce(() -> offset -= 0.5);
    }

    public double getOffset() {
        return offset;
    }

    // =========================================================================
    // Targeting Logic
    // =========================================================================

    /**
     * Calculates aiming parameters for the Central Hub while moving. Relies on RobotTracker to
     * project the robot's pose forward based on velocity.
     */
    public AimingParameters shootAtHubOnTheMove() {
        return shootAtPoseOnTheMove(robotTracker.getHub());
    }

    /**
     * Calculates aiming parameters for a specific target pose while moving. * Logic: 1. Get the
     * "Compensated Pose" from RobotTracker (where the robot will be). 2. Calculate vector from
     * Compensated Robot -> Target. 3. Align robot heading to this vector. 4. Lookup hood angle
     * based on the distance of this vector.
     */
    public AimingParameters shootAtPoseOnTheMove(Pose2d targetPose) {
        // 1. Compensate for robot velocity
        Pose2d shotPose = robotTracker.compensate(robotTracker.getPose());

        // 2. Calculate vector from Projected Robot Position to Target
        Translation2d robotToTargetTranslation =
                targetPose.getTranslation().minus(shotPose.getTranslation());

        // 3. Calculate Distances
        double compensatedDistance = robotToTargetTranslation.getNorm();

        // 4. Calculate Desired Heading (Yaw)
        Rotation2d aimHeading = robotToTargetTranslation.getAngle();

        // 5. Calculate Hood Angle
        // We use the distance from the compensated position to the target.
        double hoodAngle = hoodAngleMap.get(compensatedDistance) + offset;

        return new AimingParameters(aimHeading, hoodAngle, kShootSpeed, compensatedDistance);
    }

    /** Calculates aiming parameters assuming the robot is stationary. */
    public AimingParameters shootAtHubStationary() {
        Pose2d currentPose = robotTracker.getPose();
        Pose2d hubPose = robotTracker.getHub();

        Translation2d robotToTarget = hubPose.getTranslation().minus(currentPose.getTranslation());
        double distance = robotToTarget.getNorm();
        Rotation2d heading = robotToTarget.getAngle();

        double hoodAngle = hoodAngleMap.get(distance) + offset;

        return new AimingParameters(heading, hoodAngle, kShootSpeed, distance);
    }

    public double getCompensatedDistance() {
        return robotTracker.getDistanceFromHub(robotTracker.compensate(robotTracker.getPose()));
    }

    // =========================================================================
    // Data Classes
    // =========================================================================

    public static class AimingParameters {
        public final Rotation2d rotation; // Field-relative robot heading
        public final double pivot; // Hood angle in degrees
        public final double shootSpeed; // Flywheel speed (m/s)
        public final double effectiveDistance; // Distance used for calculation

        public AimingParameters(
                Rotation2d rotation, double pivot, double shootSpeed, double effectiveDistance) {
            this.rotation = rotation;
            this.pivot = pivot;
            this.shootSpeed = shootSpeed;
            this.effectiveDistance = effectiveDistance;
        }

        public AimingParameters withShootSpeed(double speed) {
            return new AimingParameters(this.rotation, this.pivot, speed, this.effectiveDistance);
        }

        public AimingParameters withRotation(Rotation2d rot) {
            return new AimingParameters(rot, this.pivot, this.shootSpeed, this.effectiveDistance);
        }

        public AimingParameters withPivot(double pivot) {
            return new AimingParameters(
                    this.rotation, pivot, this.shootSpeed, this.effectiveDistance);
        }

        @Override
        public String toString() {
            return String.format(
                    "AimingParams[Rot: %.1f deg, Hood: %.1f deg, Spd: %.1f, Dist: %.2fm]",
                    rotation.getDegrees(), pivot, shootSpeed, effectiveDistance);
        }
    }
}
