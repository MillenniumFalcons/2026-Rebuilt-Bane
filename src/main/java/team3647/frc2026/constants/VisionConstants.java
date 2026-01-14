package team3647.frc2026.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {
    public static final AprilTagFieldLayout k2025AprilTags =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final Vector<N3> baseStdDevs = VecBuilder.fill(0.05, 0.05, 0.1);

    // public static final String kCrossbarLLName = "limelight-xbar";
    // public static final String kIntakeLLName = "limelight-coral";
    // public static final String backRightCamName = "backRight";
    // public static final String frontRightCamName = "frontRight";
    // public static final String frontLeftCamName = "frontLeft";
    // public static final String backLeftCamName = "backLeft";
    // public static final String frontLLName = "limelight-front";

    // redesign cameras
    public static final String kRightCam = "limelight-right";
    public static final String kLeftCam = "limelight-left";
    public static final String kCrossbarCam = "limelight-intake";

    // redesign transforms

    public static final Transform3d rightMount =
            new Transform3d(
                    Inches.of(6.75),
                    Inches.of(9),
                    Inches.of(15),
                    new Rotation3d(Degrees.of(0), Degrees.of(-19.653), Degrees.of(0)));

    public static final Transform3d leftMount =
            new Transform3d(
                    Inches.of(6.75),
                    Inches.of(-9),
                    Inches.of(15),
                    new Rotation3d(Degrees.of(0), Degrees.of(-19.653), Degrees.of(0)));

    // public static final Transform3d LLsideMount =
    //         new Transform3d(
    //                 Inches.of(3.68096),
    //                 Inches.of(-7.92081),
    //                 Inches.of(27.09660),
    //                 new Rotation3d(Degrees.of(0), Degrees.of(-55.018), Degrees.of(-23.08)));

    // public static final Transform3d LLCrossMount =
    //         new Transform3d(
    //                 Inches.of(-2.22398),
    //                 Inches.of(0),
    //                 Inches.of(33.5793),
    //                 new Rotation3d(Degrees.of(0), Degrees.of(18.718), Degrees.of(180)));

    // public static final Transform3d FrontRight =
    //         new Transform3d(
    //                 Inches.of(11.466857),
    //                 Inches.of(-9.67842),
    //                 Inches.of(7.37611),
    //                 new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(31.67)));

    // public static final Transform3d FrontLeft =
    //         new Transform3d(
    //                 Inches.of(11.527214),
    //                 Inches.of(9.726327),
    //                 Inches.of(7.357755),
    //                 new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(-31.67)));

    // public static final Transform3d BackRight =
    //         new Transform3d(
    //                 Inches.of(-9.34771),
    //                 Inches.of(-10.574810),
    //                 Inches.of(8.084923),
    //                 new Rotation3d(Degrees.of(0), Degrees.of(-35.65), Degrees.of(-90 - 27)));

    // public static final Transform3d BackLeft =
    //         new Transform3d(
    //                 Inches.of(-2.87203),
    //                 Inches.of(8.35946),
    //                 Inches.of(28.72351),
    //                 new Rotation3d(Degrees.of(11.797), Degrees.of(10), Degrees.of(65)));
    // public static final Transform3d FrontLL =
    //         new Transform3d(
    //                 Inches.of(12.75),
    //                 Inches.of(0),
    //                 Inches.of(1.5),
    //                 new Rotation3d(0, Math.toRadians(-24.529), 0));
}
