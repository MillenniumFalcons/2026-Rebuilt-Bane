package team3647.frc2026.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import team3647.frc2026.constants.FieldConstants;

public class AllianceFlip {

    public static Pose2d flip(Pose2d pose, Alliance color) {
        if (color == Alliance.Blue) return pose;
        return new Pose2d(
                FieldConstants.kFieldLength.minus(pose.getMeasureX()),
                FieldConstants.kFieldWidth.minus(pose.getMeasureY()),
                pose.getRotation().rotateBy(Rotation2d.kPi));
    }

    public static Pose2d flip(Pose2d pose) {
        return flip(pose, Alliance.Red);
    }

    public static Translation2d flip(Translation2d trans) {
        return new Translation2d(
                FieldConstants.kFieldLengthM - trans.getX(),
                FieldConstants.kFieldWidthM - trans.getX());
    }

    public static double flipX(double x, Alliance color) {
        if (color == Alliance.Blue) return x;
        return FieldConstants.kFieldLengthM - x;
    }

    public static double flipY(double y, Alliance color) {
        if (color == Alliance.Blue) return y;
        return FieldConstants.kFieldWidthM - y;
    }

    public static Rotation2d flipRot(Rotation2d rot, Alliance color) {
        if (color == Alliance.Blue) return rot;
        return rot.rotateBy(Rotation2d.kPi);
    }
}
