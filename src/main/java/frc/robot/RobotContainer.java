// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Claw.CloseClaw;
import frc.robot.commands.Claw.OpenClaw;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TeleopPositionArm;
import frc.robot.commands.arm.AddOffsetJointOne;
import frc.robot.commands.arm.AddOffsetJointTwo;
import frc.robot.commands.arm.SubOffsetJointOne;
import frc.robot.commands.arm.SubOffsetJointTwo;
import frc.robot.config.ArmConfiguration;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

import java.util.List;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final ClawSubsystem clawSubsystem = new ClawSubsystem();
    private final Joystick driverJoystick = new Joystick(OIConstants.kDriverControllerPort);
    private final XboxController xboxController = new XboxController(2);
    public RobotContainer() {
        /*
        *By using new to create a new instance of the command, you are creating a separate object that encapsulates the
         command's behavior and allows it to be scheduled, initialized, and executed independently of other instances of
         the same command.
         */
        // setting joystick commands to default commands for the swerve chassis
        // using lambda expression to pass the values, so they can use the get() method for x & y & turning speeds
        swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                swerveSubsystem,
                /*
                The getRawAxis() method returns a double value representing the position or value of the specified axis.
                The range of the returned value typically depends on the specific joystick or gamepad being used, but
                it is often in the range of -1.0 to 1.0. A value of 0.0 indicates that the axis is centered or not being
                pushed in any direction, while positive and negative values indicate the axis being pushed in a
                particular direction.
                 */
                // using lambdas to create new supplier object and providing implementations for get() method
                () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                // by default will operate in the field's reference frame
                () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

        armSubsystem.setDefaultCommand(new TeleopPositionArm(armSubsystem));

        // Configure the trigger bindings
        configureBindings();
    }

    // this method will reset the robot's heading. resets the direction of the field's reference frame
    private void configureBindings() {
        /*

         */
        // creates a new joystick button on the gamepad (button 2)
        // use lambda here so we don't need to create another conditional flow

        // the lambda expression is used as an argument
        new JoystickButton(driverJoystick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());

        /*
        we use :: to make method references while we use lambda expressions to make anonymous function calls to pass
        in the appropriate arguments

        we use :: instead of . when creating Trigger object because if we use . operator, it will just make a method
        call to xBoxController.getButtonPressed and return false. We use armSubsystem.set...position because we want
        to make a method reference instead of calling it.

        When the x and a button triggers .onTrue returns true, we use lambda expressions to make an anonymous function
        call and pass in the value.
         */
        Trigger yButtonTrigger = new Trigger(xboxController::getYButtonPressed);
        yButtonTrigger.onTrue(new RunCommand(armSubsystem::setNextPosition));

        Trigger bButtonTrigger = new Trigger(xboxController::getBButtonPressed);
        bButtonTrigger.onTrue(new RunCommand(armSubsystem::setPreviousPosition));

        Trigger xButtonTrigger = new Trigger(xboxController::getXButtonPressed);
        xButtonTrigger.onTrue(new RunCommand(() -> armSubsystem.setTargetPosition(ArmConfiguration.Positions.FLOOR)));

        Trigger aButtonTrigger = new Trigger(xboxController::getAButtonPressed);
        aButtonTrigger.onTrue(new RunCommand(() -> armSubsystem.setTargetPosition(ArmConfiguration.Positions.HOME)));

        // creates a new trigger object (when trigger objects are first created, they always return false)
        // creates a new trigger object with lambda expression that returns true if POV is read as 0
        // using lambdas to create a supplier object -> providing implementation for get() method.
        Trigger addOffsetJointOneTrigger = new Trigger(() -> xboxController.getPOV() == 0);
        addOffsetJointOneTrigger.onTrue(new AddOffsetJointOne(armSubsystem));

        Trigger addOffsetJointTwoTrigger = new Trigger(() -> xboxController.getPOV() == 90);
        addOffsetJointTwoTrigger.onTrue(new AddOffsetJointTwo(armSubsystem));

        Trigger subOffsetJointOneTrigger = new Trigger(() -> xboxController.getPOV() == 180);
        subOffsetJointOneTrigger.onTrue(new SubOffsetJointOne(armSubsystem));

        Trigger subOffsetJointTwoTrigger = new Trigger(() -> xboxController.getPOV() == 270);
        subOffsetJointTwoTrigger.onTrue(new SubOffsetJointTwo(armSubsystem));

        // Claw
        Trigger openClaw = new Trigger(() -> xboxController.getLeftBumperPressed());
        openClaw.onTrue(new OpenClaw(clawSubsystem));

        Trigger closeClaw = new Trigger(() -> xboxController.getRightBumperPressed());
        closeClaw.onTrue(new CloseClaw(clawSubsystem));
    }

    public Command getAutonomousCommand() {
        // 1. Create trajectory settings (specifying how fast we want the robot to drive)
        /*
            setKinematics(): Adds a differential drive kinematics constraint to ensure that no wheel velocity of a
                             differential drive goes above the max velocity.
         */
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);

        // 2. Generate trajectory (Think of it as a coordinates plane)
        /*
            Three arguments: Pose2d, List of translation2d, post2d, and trajectoryConfig

            a. First Pose2d argument is the initial point
            b. List.of(Translation2d) is the points the robot will go through
            c. Second Pose2d is the final point, and tells the robot to spin 180 degrees
            d. Trajectory configuration
         */
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        new Translation2d(1, 0),
                        new Translation2d(1, -1)
                ),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig
        );

        // 3. Define PID controllers for tracking trajectory (corrects for errors in the trajectory)
        // -> proportional term is good enough here
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        // ProfiledPIDController is just a pid controller with a limit on its maximum speed and acceleration
        // -> So the robot rotates slowly over the course of its trajectory
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstrains);
        // since robot heading is continuous (between -180 to 180), we will make thetaController continuous as well
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // :: references a method without invoking them, so they can be passed into the constructor and be called later
        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand= new SwerveControllerCommand(
                // trajectory
                trajectory,
                // function to get robot coordinates
                swerveSubsystem::getPose,
                // chassis kinematics,
                DriveConstants.kDriveKinematics,
                // three pid controllers
                xController,
                yController,
                thetaController,
                // function to set swerve module states
                swerveSubsystem::setModuleStates,
                // subsystem to "require"
                swerveSubsystem);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                /*
                    by resetting odometer like this, even if robot is not on the initial point of our trajectory,
                    it will  move the trajectory to the robot's current location
                 */
                // instant commands executes and finishes immediately
                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> swerveSubsystem.stopModules())
        );
    }
}
