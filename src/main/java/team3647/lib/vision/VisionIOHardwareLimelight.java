package team3647.lib.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.concurrent.atomic.AtomicReference;
import team3647.frc2026.constants.VisionConstants;

/**
 * Hardware implementation of VisionIO using Limelight cameras.
 *
 * <p>Based on Team 254's 2025 vision architecture.
 */
public class VisionIOHardwareLimelight implements VisionIO {
    NetworkTable tableA =
            NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightATableName);
    NetworkTable tableB =
            NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightBTableName);
    NetworkTable tableC =
            NetworkTableInstance.getDefault().getTable(VisionConstants.kLimelightCTableName);
    VisionState visionState;
    AtomicReference<VisionIOInputs> latestInputs = new AtomicReference<>(new VisionIOInputs());

    private static final double[] DEFAULT_STDDEVS =
            new double[VisionConstants.kExpectedStdDevArrayLength];

    public VisionIOHardwareLimelight(VisionState visionState) {
        this.visionState = visionState;
        setLLSettings();
    }

    private void setLLSettings() {
        double[] cameraAPose = {
            VisionConstants.kRobotToCameraAForward,
            VisionConstants.kRobotToCameraASide,
            VisionConstants.kCameraAHeightOffGroundMeters,
            0.0,
            VisionConstants.kCameraAPitchDegrees,
            VisionConstants.kCameraAYawOffset.getDegrees()
        };

        tableA.getEntry("camerapose_robotspace_set").setDoubleArray(cameraAPose);

        double[] cameraBPose = {
            VisionConstants.kRobotToCameraBForward,
            VisionConstants.kRobotToCameraBSide,
            VisionConstants.kCameraBHeightOffGroundMeters,
            0.0,
            VisionConstants.kCameraBPitchDegrees,
            VisionConstants.kCameraBYawOffset.getDegrees()
        };

        tableB.getEntry("camerapose_robotspace_set").setDoubleArray(cameraBPose);

        double[] cameraCPose = {
            VisionConstants.kRobotToCameraCForward,
            VisionConstants.kRobotToCameraCSide,
            VisionConstants.kCameraCHeightOffGroundMeters,
            0.0,
            VisionConstants.kCameraCPitchDegrees,
            VisionConstants.kCameraCYawOffset.getDegrees()
        };

        tableC.getEntry("camerapose_robotspace_set").setDoubleArray(cameraCPose);
    }

    @Override
    public void readInputs(VisionIOInputs inputs) {
        readCameraData(tableA, inputs.cameraA, VisionConstants.kLimelightATableName);
        readCameraData(tableB, inputs.cameraB, VisionConstants.kLimelightBTableName);
        readCameraData(tableC, inputs.cameraC, VisionConstants.kLimelightCTableName);
        latestInputs.set(inputs);
    }

    private void readCameraData(
            NetworkTable table, VisionIOInputs.CameraInputs camera, String limelightName) {
        camera.seesTarget = table.getEntry("tv").getDouble(0) == 1.0;
        if (camera.seesTarget) {
            try {
                var maybeMegatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
                var maybeRobotPose3d =
                        LimelightHelpers.toPose3D(
                                LimelightHelpers.getBotPose_wpiBlue(limelightName));

                if (maybeMegatag.isPresent()) {
                    var megatag = maybeMegatag.get();
                    camera.megatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
                    camera.megatagCount = megatag.tagCount;
                    camera.fiducialObservations =
                            FiducialObservation.fromLimelight(megatag.rawFiducials);
                }
                maybeRobotPose3d.ifPresent(p -> camera.pose3d = p);

                camera.standardDeviations =
                        table.getEntry("stddevs").getDoubleArray(DEFAULT_STDDEVS);
            } catch (Exception e) {
                System.err.println("Error processing Limelight data: " + e.getMessage());
            }
        }
    }
}
