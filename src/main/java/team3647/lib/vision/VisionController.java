// package team3647.lib.vision;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import java.util.ArrayList;
// import java.util.Collections;
// import java.util.function.Consumer;
// import java.util.function.Function;
// import org.littletonrobotics.junction.Logger;
// import team3647.lib.team6328.VirtualSubsystem;
// import team3647.lib.team9442.AutoChooser;

// public class VisionController extends VirtualSubsystem {
//     private final AprilTagCamera[] cameras;
//     private final Consumer<VisionMeasurement> botPoseAcceptor;
//     private final Consumer<Pose2d> resetPose;
//     private final Function<VisionMeasurement, Boolean> shouldAddData;
//     private final ArrayList<VisionMeasurement> list = new ArrayList<>();
//     public static boolean hasReset = false;
//     private int count;

//     public VisionController(
//             Consumer<VisionMeasurement> visionAcceptor,
//             Function<VisionMeasurement, Boolean> shouldAddData,
//             Consumer<Pose2d> resetPose,
//             AprilTagCamera... cameras) {
//         this.cameras = cameras;
//         this.resetPose = resetPose;
//         this.shouldAddData = shouldAddData;
//         this.botPoseAcceptor = visionAcceptor;
//     }

//     @Override
//     public void periodic() {

//         list.clear();

//         for (AprilTagCamera camera : cameras) {

//             var inputs = camera.QueueToInputs();

//             if (inputs.isEmpty()) {
//                 continue;
//             }

//             var getInputs = inputs.get();

//             if (shouldAddData.apply(getInputs)) {
//                 list.add(getInputs);

//                 count = 0;

//             } else {
//                 count++;
//                 // && AutoChooser.selected (add this back)
//                 if (count > 4) {

//                     resetPose.accept(getInputs.pose);
//                     hasReset = true;
//                     Logger.recordOutput("Robot/Reset", getInputs.pose);

//                     break;
//                 }
//             }
//             Logger.recordOutput("selected?", AutoChooser.selected);

//             // Logger.recordOutput("Robot/" + camera.getName(), getInputs.pose);
//         }

//         if (!list.isEmpty()) {

//             Collections.sort(list);

//             botPoseAcceptor.accept(list.get(0));

//             Logger.recordOutput("Robot/Vision", list.get(0).pose);
//             Logger.recordOutput("Robot/Camera", list.get(0).name);

//             // Logger.recordOutput("stddev", list.get(0).stdDevs.get(0, 0));
//         } else {
//             Logger.recordOutput("Robot/Vision", new Pose2d(1, 1, new Rotation2d()));
//             // Logger.recordOutput("stddev", 0.0);
//         }
//         SmartDashboard.putBoolean(
//                 "Has Vision?",
//                 list.isEmpty() || !list.get(0).pose.equals(new Pose2d(1, 1, new Rotation2d())));
//     }
// }
