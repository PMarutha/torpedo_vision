package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


public final class DriveTrain {
    private DifferentialDrive drivetrain;

    private DifferentialDrivePoseEstimator poseEstimator; 
    private DifferentialDriveKinematics tank_kinematics;

    private RelativeEncoder leftEncoder, rightEncoder;

    private AHRS gyro;


    public DriveTrain() {
        CANSparkMax Left1 = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_PORTS[0], MotorType.kBrushless);
        CANSparkMax Left2 = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_PORTS[1], MotorType.kBrushless);
        CANSparkMax Left3 = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_PORTS[2], MotorType.kBrushless);
        CANSparkMax Right1 = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_PORTS[0], MotorType.kBrushless);
        CANSparkMax Right2 = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_PORTS[1], MotorType.kBrushless);
        CANSparkMax Right3 = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_PORTS[2], MotorType.kBrushless);
       
        Left1.setIdleMode(IdleMode.kBrake);
        Left2.setIdleMode(IdleMode.kBrake);
        Left3.setIdleMode(IdleMode.kBrake);
        Right1.setIdleMode(IdleMode.kBrake);
        Right2.setIdleMode(IdleMode.kBrake);
        Right3.setIdleMode(IdleMode.kBrake);

        Left1.setSmartCurrentLimit(Constants.CURRENT_LIMIT_AMPS);
        Left1.setSecondaryCurrentLimit(Constants.CURRENT_LIMIT_AMPS);
        Left2.setSmartCurrentLimit(Constants.CURRENT_LIMIT_AMPS);
        Left2.setSecondaryCurrentLimit(Constants.CURRENT_LIMIT_AMPS);
        Left3.setSmartCurrentLimit(Constants.CURRENT_LIMIT_AMPS);
        Left3.setSecondaryCurrentLimit(Constants.CURRENT_LIMIT_AMPS);
        Right1.setSmartCurrentLimit(Constants.CURRENT_LIMIT_AMPS);
        Right1.setSecondaryCurrentLimit(Constants.CURRENT_LIMIT_AMPS);
        Right2.setSmartCurrentLimit(Constants.CURRENT_LIMIT_AMPS);
        Right2.setSecondaryCurrentLimit(Constants.CURRENT_LIMIT_AMPS);
        Right3.setSmartCurrentLimit(Constants.CURRENT_LIMIT_AMPS);
        Right3.setSecondaryCurrentLimit(Constants.CURRENT_LIMIT_AMPS);

        rightEncoder = Right1.getEncoder();
        leftEncoder = Left1.getEncoder();

        gyro = new AHRS(SPI.Port.kMXP);

        var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        var visionStdDevs = VecBuilder.fill(1, 1, 1);

        poseEstimator = new DifferentialDrivePoseEstimator(tank_kinematics, gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), new Pose2d(), stateStdDevs, visionStdDevs);

        final MotorControllerGroup leftMotors = new MotorControllerGroup(Left1, Left2, Left3);
        final MotorControllerGroup rightMotors = new MotorControllerGroup(Right1, Right2, Right3);
        this.drivetrain = new DifferentialDrive(leftMotors, rightMotors);
    }

    public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
    }

    public void updatePoseEstimator(){
        poseEstimator.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }

    public double getLeftPosition(){
        return leftEncoder.getPosition();
    }

    public double getRightPosition(){
        return rightEncoder.getPosition();
    }

    public Rotation2d getAngle(){
        return gyro.getRotation2d();
    }

    public void updateOdometry() {
        poseEstimator.update(getAngle(), getLeftPosition(), getRightPosition());
    }



    public void run(){
        final double rSpeed = Math.signum(Robot.driverController.getRightY())*Constants.DRIVE_POWER*Math.pow(Math.abs(Robot.driverController.getRightY()), Constants.DRIVE_EXPONENT);
       //(sign of input) * (drive power) * (absolute value of input)^(drive exponent)
       final double lSpeed = Math.signum(Robot.driverController.getLeftY())*Constants.DRIVE_POWER*Math.pow(Math.abs(Robot.driverController.getLeftY()), Constants.DRIVE_EXPONENT);
       //(sign of input) * (drive power) * (absolute value of input)^(drive exponent)
        drivetrain.tankDrive(-1 * lSpeed, rSpeed);
    }
}