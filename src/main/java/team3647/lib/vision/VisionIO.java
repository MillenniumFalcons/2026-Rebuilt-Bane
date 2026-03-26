package team3647.lib.vision;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Hardware abstraction interface for vision systems.
 *
 * <p>Based on Team 254's 2025 vision architecture.
 */
public interface VisionIO {

    class VisionIOInputs {
        public static class CameraInputs {
            public boolean seesTarget;
            public FiducialObservation[] fiducialObservations = new FiducialObservation[0];
            public MegatagPoseEstimate megatagPoseEstimate;
            public MegatagPoseEstimate megatag2PoseEstimate;
            public int megatag2Count;
            public int megatagCount;
            public Pose3d pose3d;
            public double[] standardDeviations =
                    new double[12]; // [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x,
            // MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
        }

        public CameraInputs cameraA = new CameraInputs();
        public CameraInputs cameraB = new CameraInputs();
        public CameraInputs cameraC = new CameraInputs();
    }

    void readInputs(VisionIOInputs inputs);
}
