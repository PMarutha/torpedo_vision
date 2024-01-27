package frc.robot;

import java.io.IOException;
import java.sql.Driver;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.LimelightHelpers.LimelightResults;
import edu.wpi.first.wpilibj.Filesystem;

public class Vision {
    private final DifferentialDriveKinematics tank_kinematics; // INSERT TRACK WIDTH HERE;
    private final DifferentialDrivePoseEstimator poseEstimator;

    private final Field2d m_field = new Field2d();
    
    // private double lastEstTimestamp = 0;

    private AprilTagFieldLayout fieldLayout;
    // private Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.45), new Rotation3d(0,0,22));
    private DriveTrain drivetrain;

    public static final Matrix<N3, N1> kTestStdDevs = VecBuilder.fill(0.2, 0.2, 1);
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);


    public Vision(DriveTrain drivetrain) {

        this.drivetrain = drivetrain;

        var stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
        var visionStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30));

        // // 2023 FIELD MAP
        // try {
        //     fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        // } catch (IOException e) {
        //     // TODO Auto-generated catch block
        //     e.printStackTrace();
        // }

        // 2024 CRESCENDO

        String bruh = Filesystem.getDeployDirectory().getAbsolutePath() + "/crescendo.json";

        try {
            fieldLayout = new AprilTagFieldLayout(bruh);
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }

        tank_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(19.5)); 
        poseEstimator = new DifferentialDrivePoseEstimator(tank_kinematics, drivetrain.getAngle(), drivetrain.getLeftPosition(), drivetrain.getRightPosition(), new Pose2d(), stateStdDevs, visionStdDevs);
        

        SmartDashboard.putData("Field", m_field);
    }


    public void visionOdometry() {
        poseEstimator.update(drivetrain.getAngle(), drivetrain.getLeftPosition(), drivetrain.getRightPosition());

        LimelightHelpers.Results results = LimelightHelpers.getLatestResults("limelight").targetingResults;
        Pose2d estPose = LimelightHelpers.toPose2D(results.botpose);

        var estStdDevs = getEstimationStdDevs(estPose);
        
       if(results.getBotPose2d().getX() > 0.5 && results.getBotPose2d().getY() > 0.5) {
         poseEstimator.addVisionMeasurement(estPose, Timer.getFPGATimestamp() - (results.latency_capture / 1000.0) - (results.latency_pipeline / 1000.0), kSingleTagStdDevs);
       }
       
        m_field.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }


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

}
