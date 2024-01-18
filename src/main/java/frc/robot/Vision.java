package frc.robot;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.LimelightHelpers.LimelightResults;
import edu.wpi.first.wpilibj.Filesystem;

public class Vision {
    private final DriveTrain drivetrain;
    private final DifferentialDriveKinematics tank_kinematics; // INSERT TRACK WIDTH HERE;
    private final DifferentialDrivePoseEstimator poseEstimator;
    
    // private double lastEstTimestamp = 0;

    private AprilTagFieldLayout fieldLayout;
    // private Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.45), new Rotation3d(0,0,22));

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);


    public Vision(){

        drivetrain = new DriveTrain();

        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        var visionStdDevs = VecBuilder.fill(1, 1, 1);

        String bruh = Filesystem.getDeployDirectory().getAbsolutePath() + "/crescendo.json";

        try {
            fieldLayout = new AprilTagFieldLayout(bruh);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        tank_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(19.5)); 
        poseEstimator = new DifferentialDrivePoseEstimator(tank_kinematics, drivetrain.getAngle(), drivetrain.getLeftPosition(), drivetrain.getRightPosition(), new Pose2d(), stateStdDevs, visionStdDevs);
        
    }


    public void visionOdometry() {
        poseEstimator.update(drivetrain.getAngle(), drivetrain.getLeftPosition(), drivetrain.getRightPosition());

        LimelightHelpers.Results results = LimelightHelpers.getLatestResults("limelight").targetingResults;
        Pose2d estPose = LimelightHelpers.toPose2D(results.botpose);

        var estStdDevs = getEstimationStdDevs(estPose);
        
        poseEstimator.addVisionMeasurement(estPose, Timer.getFPGATimestamp() - (results.latency_capture / 1000.0) - (results.latency_pipeline / 1000.0), estStdDevs);
    }


    // IGNORE ------------------------------------------------------------------------------------------------
    // public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    //     var visionEst = photonEstimator.update();
    //     double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
    //     boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

    //     if (newResult) lastEstTimestamp = latestTimestamp;
    //     return visionEst;
    // }
    // -----------------------------------------------------------------------------------------------

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = kSingleTagStdDevs;
        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults("limelight");
        var targets = results.targetingResults.targets_Fiducials;
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = fieldLayout.getTagPose((int) tgt.fiducialID);
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    // IGNORE ------------------------------------------------------------------------------------------------------
    // var visionEst = vision.getEstimatedGlobalPose();
    // visionEst.ifPresent(
    //     est -> {
    //       var estPose = est.estimatedPose.toPose2d();
    //       // Change our trust in the measurement based on the tags we can see
    //       var estStdDevs = vision.getEstimationStdDevs(estPose);

    //       drivetrain.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
    //             });
    // -------------------------------------------------------------------------------------------------------------

}
