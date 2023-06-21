package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRighturningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);
    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    // gyroscope
    private final ADIS16470_IMU adis16470Imu = new ADIS16470_IMU();

    // odometer
    // passes in swerve drive kinematics configurations and current gyroscope angle
    // *here, the gyro angle has not yet been resetted, so we pass in 0 degrees by creating new Rotation2d instance with 0
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), getModulePositions());

    /*
        creating a new thread makes sure that zeroHeading() is waiting to be executed in the background (another thread)
        while the rest of the constructor method is executing
     */
    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
                System.out.println(e.getMessage());
            }
        }).start();
    }

    // this method will reset the gyroscope everytime the robot boots up -> sets current direction as forward direction
    public void zeroHeading() {
        adis16470Imu.reset();
    }

    // this method will get the robot's heading from the gyro scope and clamp it between -180 to 180 degree for easy use
    // *By default, the gyro scope value is continuous, meaning it can go up to 360 degrees, 720 degrees, etc..
    public double getHeading() {
        return Math.IEEEremainder(adis16470Imu.getAngle(), 360);
    }

    // wpi library often wants things in rotation2d, so we convert getHeading()'s value into rotation 2d
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    // this method will get the location determined by the odometer
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    // this method will reset the odometer to a new location
    public void resetOdometry (Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), getPose());
    }

    // monitoring the robot's heading value from the periodic function
    @Override
    // periodic is called 50 times per second, used to update information and reading sensor inputs
    public void periodic() {
        odometer.update(getRotation2d(), getModulePositions());
        //monitoring robot's heading
        SmartDashboard.putNumber("Robot Heading", getHeading());
        // monitoring robot's coordinates
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    }

    // this method will stop the modules (using method from SwerveModule.java)-stops velocity of turning and drive motor
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        SwerveModule[] modArray = {frontLeft, frontRight, backLeft, backRight};
        for (int i = 0; i <= 4; i++) {
            positions[i] = modArray[i].getPosition();
        }
        return positions;
    }

    // setting modules
    // this method will take in an array of all four modules and set their states
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        /*
            if all four wheels are driven with max speed, we lose control of the robot's steering, so we decrease all
            wheel speeds proportionally until they are 'achievable'
         */
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}
