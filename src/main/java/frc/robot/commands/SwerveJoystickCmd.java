package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.DriveConstants;

import java.util.function.Supplier;

public class SwerveJoystickCmd extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    // suppliers have one method - get()
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    // Constructor method
    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction,
            Supplier<Double> ySpdFunction,
            Supplier<Double> turningSpdFunction,
            Supplier<Boolean> fieldOrientedFunction ) {

        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        // sets the dependencies for subsystem to enable proper... stuff? (resource management and coordination)
        // *Can not schedule or run commands with adding requirements
        addRequirements(swerveSubsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    // called 50 times per second
    public void execute() {
        // 1. get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // 2. apply deadband -> if joystick doesn't center back to 0, we will ignore any small inputs to protect motors
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. make driving smoother -> if joysticks is pushed too violently, we don't want a sudden huge acceleration
        // 3.5 scaling speed down by 1/4
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        // if fieldOriented is true, we'll ask wpi library to convert to the robot's local reference frame

        /*
            1. Relative to the field: The robot is driving relative to the field's coordinate system, regardless of
               robot's orientation. Driving forward and sideways is the field's x and y coordinates, no matter where
               the robot is facing.

            2. Relative to the robot itself: The robot is driving relative to its own reference frames. It is driving
               according to its orientation and the x & y coordinates are switching as its orientation changes.

            Driving relative to the field is usually used for autonomous period while driving relative to the robot
            itself is usually used for teleoperated.
         */
        if (fieldOrientedFunction.get()) {
            // relative to field -> requires xSpeed, ySpeed, turningSpeed, and current robot heading
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed,
                                                                  swerveSubsystem.getRotation2d());
        } else {
            // relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. convert chassis speeds to individual module states -> generates an array of four swerve module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
