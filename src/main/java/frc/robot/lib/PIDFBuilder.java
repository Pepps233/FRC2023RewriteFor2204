package frc.robot.lib;

import com.pathplanner.lib.auto.PIDConstants;

public class PIDFBuilder {
    private final double proportional;
    private final double integral;
    private final double derivative;
   // private final double filtered;

    public PIDFBuilder(double proportional, double integral, double derivative) {
        this.proportional = proportional;
        this.integral = integral;
        this.derivative = derivative;
    }

    public double getProportional() {
        return proportional;
    }

    public double getIntegral() {
        return integral;
    }
    public double getDerivative() {
        return derivative;
    }
}
