package team3647.frc2026.constants;

import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import java.util.List;

public class AutoConstants {

    public static SwerveSample kEmptySample =
            new SwerveSample(-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, new double[] {0}, new double[] {0});
    public static EventMarker kEmptyEventMarker = new EventMarker(-1, "nothing");

    public static Trajectory<SwerveSample> kEmptyTraj =
            new Trajectory<SwerveSample>(
                    "kEmtpy", List.of(kEmptySample), List.of(1), List.of(kEmptyEventMarker));

    public static final PIDController xController = new PIDController(1.5, 0, 0);
    public static final PIDController yController = new PIDController(2.5, 0, 0.015);
    public static final ProfiledPIDController profiledRotController =
            new ProfiledPIDController(4.5, 0, 0.045, new Constraints(5, 5));
    public static final PIDController rotController = new PIDController(1.25, 0, 0);
    public static final PIDController autoXController = new PIDController(2.0, 0, 0.05);
    public static final PIDController autoYController = new PIDController(2.0, 0, 0.05);

    public static final double kEndRadius = 0.3;
    public static final double kEndAngle = 360;

    /**
     * Max age (s) of a vision fix for it to count toward ending an auto path at its endpoint. Kept
     * loose: the gate is only meant to reject completion with no recent vision at all.
     */
    public static final double kVisionEndpointMaxAgeSeconds = 1.0;

    /**
     * Radius (m) a vision fix must be within of the endpoint before an auto path may finish. Kept
     * loose so a roughly-correct vision fix is enough to allow completion.
     */
    public static final double kVisionEndpointRadiusMeters = 0.4;

    /**
     * Grace period (s) after the trajectory ends during which, if no vision fix arrives but
     * odometry still places the robot at the endpoint, the path is allowed to finish anyway.
     */
    public static final double kVisionEndpointFallbackSeconds = 1.0;

    public static final double kFallbackRadius = 0.5;

    //     public static final TunablePID tunableAutoXController =
    //             new TunablePID("Auto/x", autoXController);
    //     public static final TunablePID tunableAutoYController =
    //             new TunablePID("Auto/y", autoYController);
}
