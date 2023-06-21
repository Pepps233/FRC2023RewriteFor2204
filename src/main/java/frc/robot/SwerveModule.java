package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.DriveConstants;
import lib.math.Conversions;

// THIS CLASS CONTROLS THE BEHAVIORS OF A SINGLE SWERVE MODULE

public class SwerveModule {

    private final TalonFX driveMotor;
    private final TalonFX turningMotor;
    private final PIDController turningPidController;
    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
                        int absoluteEncoderId, double absoluteEncoderOffsetRad, boolean absoluteEncoderReversed) {

        this.absoluteEncoder = new AnalogInput(absoluteEncoderId);
        this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRad;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new TalonFX(turningMotorId);

        // if motor is reversed (ex. read as backwards), invert it back
        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        // both encoders will be set in Pid slot 0, (will be controlled by same pid values)
        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, ModuleConstants.timeOutMs);
        turningMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,ModuleConstants.timeOutMs);

        // convert encoders' position and velocity from rotations to meters per second and radians

        //conversionDriveMotor();
        //conversionTurningMotor();

        // the kP value is already enough for moving the angle motor
        turningPidController = new PIDController(ModuleConstants.kP, 0, 0);
        // sets pid to be circular like a circle
        turningPidController.enableContinuousInput(-Math.PI,Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotor.getSelectedSensorPosition() * ModuleConstants.PositionsToRadiansConversion;
    }

    public double getTurningPosition() {
        return turningMotor.getSelectedSensorPosition() * ModuleConstants.PositionsToRadiansConversion;
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity() * ModuleConstants.FalconToMetersPerSecondConversion;
    }

    public double getTurningVelocity() {
        return turningMotor.getSelectedSensorVelocity() * ModuleConstants.FalconToMetersPerSecondConversion;
    }

    // some methods will request turning positions as Rotation2d
    public Rotation2d turningPositionRadiansToRotation2d() {
        return Rotation2d.fromRadians(getTurningPosition());
    }

    // gets value of absolute encoder
    public double getAbsoluteEncoderRad() {
        /*
            1. divide the absolute encoder's voltage reading by the voltage we are supplying it with -> gives us how many
            percent of a full rotation it is reading.
            2. then multiply angle by 2PI to convert it into radians
            3. subtract angle by the offset to get the actual wheel angle
            4. multiply by -1 if it is reversed
         */

        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return absoluteEncoderReversed? angle * -1 : angle;
    }

    // this method will reset the drive encoder readings and set turning motor's value to absolute encoder so the
    // reading is aligned with the wheel's actual angle
    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turningMotor.setSelectedSensorPosition(getAbsoluteEncoderRad());
    }

    // parameters of SwerveModuleState() -> drive velocity in meters per second and rotation2d angle (provide radians)
    /*
       the 'new' ensures that we are creating a new instance of class SwerveModuleState and Rotation2d everytime
       getState() is called, since we are working with four different swerve modules.
    */
    /*
        *We need this method because some places requires SwerveModule's information to be used in form
        of SwerveModuleState
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /*
        this method will actuate the module, which takes in a SwerveModuleState object that has a velocity request
        and an angle request

        state represents desired state, getState().angle represents that we are accessing the angle instead of
        drive velocity of the returned values
     */
    public void setDesiredState(SwerveModuleState state) {
        /*
            avoiding bug where wheels will go back to default positioning after letting go of joysticks:
            if the new command scheduled has no significant change in velocity, we ignore the command and stop the
            motors from moving and exit the setDesiredState() function.
         */
        if(Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }

        // optimize angle setpoint so "it" never needs to move more than 90 degrees
        state = SwerveModuleState.optimize(state,getState().angle);
        // scaling velocity down using robot's max speed and sets it to the drive motor
        // *open loop*
        driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond / DriveConstants.MaxSpeedMetersPerSecond);
        /*
            for turning motor, we use pid controller to calculate the angle's setpoint and the current position
            in radians
         */
        turningMotor.set(ControlMode.Position, turningPidController.calculate(getDrivePosition(),state.angle.getRadians()));
        // setting debug information
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    // this method constructs a swerve module position
    public SwerveModulePosition getPosition() {
        // 'new' is used here because it is returning a different module's position (four swerve modules)
        return new SwerveModulePosition(Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(),
                Constants.wheelCircumference, Constants.gearRatio), Rotation2d.fromRadians(getTurningPosition()));
    }

    // this method is used to stop the motors
    public void stop() {
        driveMotor.set(ControlMode.Velocity, 0);
        turningMotor.set(ControlMode.Velocity, 0);
    }

}
