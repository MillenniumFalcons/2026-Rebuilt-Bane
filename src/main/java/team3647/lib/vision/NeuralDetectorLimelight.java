// package team3647.lib.vision;

// import java.awt.geom.Line2D;
// import java.util.Arrays;
// import team3647.lib.vision.LimelightHelpers.RawDetection;

// public class NeuralDetectorLimelight implements NeuralDetector {
//     private final String name;

//     public NeuralDetectorLimelight(String name) {
//         this.name = name;
//     }

//     @Override
//     public double getTX() {
//         return LimelightHelpers.getTX(name);
//     }

//     @Override
//     public double getTY() {
//         return LimelightHelpers.getTY(name);
//     }

//     @Override
//     public double getTTheta() {
//         var detections = LimelightHelpers.getRawDetections(name);
//         RawDetection result;
//         Line2D[] sides;
//         Line2D side;
//         double angle = 0;

//         if (detections.isPresent()) {
//             if (detections.get().length < 1) return 0;
//             result = detections.get()[0];
//             sides =
//                     new Line2D[] {
//                         new Line2D.Double(
//                                 result.corner0_X,
//                                 result.corner0_Y,
//                                 result.corner1_X,
//                                 result.corner1_Y),
//                         new Line2D.Double(
//                                 result.corner1_X,
//                                 result.corner1_Y,
//                                 result.corner2_X,
//                                 result.corner2_Y),
//                         new Line2D.Double(
//                                 result.corner2_X,
//                                 result.corner2_Y,
//                                 result.corner0_X,
//                                 result.corner0_Y)
//                     };

//             Arrays.sort(
//                     sides,
//                     (Line2D l1, Line2D l2) ->
//                             Double.compare(
//                                     l1.getP1().distance(l1.getP2()),
//                                     l2.getP1().distance(l2.getP2())));

//             side = sides[1];
//             try {
//                 angle = Math.atan((side.getY1() - side.getY2()) / (side.getX1() - side.getX2()));
//             } catch (Exception e) {

//             }
//         }
//         return angle;
//     }

//     @Override
//     public boolean hasTarget() {
//         var detections = LimelightHelpers.getRawDetections(name);
//         return detections.isPresent();
//     }
// }
