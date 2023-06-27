package frc.robot.homing;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.Pair;
import frc.robot.lib.PIDFBuilder;

public class HomingConfiguration {
    public final String name;
    public final int homingSwitchId;
    public final double homingPower;
    public final double slewRate;
    public final PIDFBuilder pidfBuilder;
    // pairs have two methods of getFirst() and getSecond()
    public final Pair<Double,Double> minMaxPower;
    public final IdleMode idleMode;
    public HomingConfiguration(String name, int homingSwitchId, double homingPower,
                               double slewRate, PIDFBuilder pidfBuilder,
                               Pair<Double,Double> minMaxPower, IdleMode idleMode) {
        this.name = name;
        this.homingSwitchId = homingSwitchId;
        this.homingPower = homingPower;
        this.slewRate = slewRate;
        this.pidfBuilder = pidfBuilder;
        this.minMaxPower = minMaxPower;
        this.idleMode = idleMode;
    }
}
