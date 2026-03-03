package team3647.frc2026.constants;

import choreo.trajectory.EventMarker;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import java.util.List;
import team3647.lib.tunables.TunablePID;

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

    public static final TunablePID tunableAutoXController =
            new TunablePID("Auto/x", autoXController);
    public static final TunablePID tunableAutoYController =
            new TunablePID("Auto/y", autoYController);
}
