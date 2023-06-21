package lib.math;

public class Conversions {
    public static double falconToMeters(double encoderPosition, double circumference, double gearRatio){
        return encoderPosition * (circumference / (gearRatio * 2048.0));
    }
}
