package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase {
    private static final PneumaticHub pcm = new PneumaticHub(ClawConstants.PNEUMATIC_HUB_ID);
    private static final DoubleSolenoid solenoid = pcm.makeDoubleSolenoid(ClawConstants.FORWARD_CHANNEL,
                                                                   ClawConstants.REVERSE_CHANNEL);
    public ClawSubsystem() {
        // enables compressor in digital mode. enables it if it is off.
        pcm.enableCompressorDigital();
    }

    public static void open() {
        solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public static void close() {
        solenoid.set(DoubleSolenoid.Value.kForward);
    }
}
