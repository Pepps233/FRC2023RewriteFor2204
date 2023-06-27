package frc.robot.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.homing.HomingConfiguration;

public class HomingNEOSparkMax{
    private static CANSparkMax sparkMax;
    private DigitalInput homeSwitch;
    private SlewRateLimiter slewRateLimiter;


    public HomingNEOSparkMax(int deviceId, HomingConfiguration homingConfiguration) {
        // brushless motors use magnets to generate motion
        sparkMax = new CANSparkMax(deviceId, CANSparkMaxLowLevel.MotorType.kBrushless);

        sparkMax.getPIDController().setP(homingConfiguration.pidfBuilder.getProportional());

        sparkMax.getPIDController().setI(homingConfiguration.pidfBuilder.getIntegral());

        sparkMax.getPIDController().setD(homingConfiguration.pidfBuilder.getDerivative());

        // limits signal range of the motor controller so we can control the speed and power and avoid mechanical stress
        // any value outside of this range will be clamped between the first and second value
        sparkMax.getPIDController().setOutputRange(homingConfiguration.minMaxPower.getFirst(),
                                                   homingConfiguration.minMaxPower.getSecond());

        // sets the idle mode of the motor to brake, meaning it can not be moved physically when idled
        sparkMax.setIdleMode(homingConfiguration.idleMode);
        // returns boolean about whether the arm is in home position through electric signal received from physical sensor
        homeSwitch = new DigitalInput(homingConfiguration.homingSwitchId);
        slewRateLimiter = new SlewRateLimiter(homingConfiguration.slewRate);
    }
    // returns true if arm is at home position
    public boolean isAtHome() {
        return homeSwitch.get();
    }
    // gets the position in rotations of the encoder
    public double getPosition() {
        return sparkMax.getEncoder().getPosition();
    }
    /*
    sets the desired position of the joints: sparkMax.getPIDController() is responsible for controlling the motor based
    on the desired setpoint. setReference(slewRateLimiter.) moves the joints at a rate limited speed to smooth out the
    movement to prevent sudden acceleration. CANSparkMax.ControlType.kPosition specifies the control mode as position
    mode -> tells the motor controller to maintain its position after meeting the desired setpoint
     */
    public void setPosition(double value) {
        sparkMax.getPIDController().setReference(slewRateLimiter.calculate(value), CANSparkMax.ControlType.kPosition);
    }
    // resets encoder position to -0.2 -> don't understand why, i should probably ask james
    public static void resetEncoder() {
        sparkMax.getEncoder().setPosition(-0.2);
    }
    public static void stop() {
        sparkMax.stopMotor();
    }
}
