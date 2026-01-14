package team3647.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.Optional;

public interface AprilTagCamera {
    public Optional<VisionMeasurement> QueueToInputs();

    public int getTagNum();

    public String getName();

    public double getTx();

    public double getTy();

    public double getTa();

    public boolean hasTarget();

    public Pose2d getTagPose();

    public Optional<Pose3d> camPose();
}
