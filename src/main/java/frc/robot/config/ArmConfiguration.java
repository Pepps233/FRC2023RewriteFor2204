package frc.robot.config;

import com.revrobotics.CANSparkMax;
import frc.robot.homing.HomingConfiguration;
import frc.robot.lib.PIDFBuilder;
import edu.wpi.first.math.Pair;

public class ArmConfiguration {
    // Spark id's and offset amount on each button press
    public static final int FIRST_JOINT_SPARK_ID = 13;
    public static final int SECOND_JOINT_SPARK_ID = 14;
    public static final double OFFSET_AMOUNT = 0.3;
    public static final HomingConfiguration FIRST_JOINT_CONFIG = new HomingConfiguration(
            "First Joint",
            1,
            0.2,
            20.0,
            new PIDFBuilder(0.15, 0.0, 0.0),
            new Pair<>(-0.3,0.3),
            CANSparkMax.IdleMode.kBrake);
    public static final HomingConfiguration SECOND_JOINT_CONFIG = new HomingConfiguration(
            "Second Joint",
            0,
            -0.1,
            20.0,
            new PIDFBuilder(0.1, 0.0, 0.0),
            new Pair<>(-0.6,0.6),
            CANSparkMax.IdleMode.kBrake);

    // enum is a way for we to keep fixed data and limited values -> like days of the week, months and etc...
    public enum Positions {
        FLOOR_TIPPED(21.0, 50.0),
        FLOOR(-2.0, 38.0),
        HOME(0.0, 0.0),
        FIRST_NODE(0.0, 36.0),
        SECOND_NODE(19.0, 7.4),
        THIRD_NODE(29.0, 4.0);
        private final double jointOnePosition;
        private final double jointTwoPosition;
        Positions(double jointOnePosition, double jointTwoPosition) {
            this.jointOnePosition = jointOnePosition;
            this.jointTwoPosition = jointTwoPosition;
        }
        // getter methods
        public double getJointOnePosition() {
            return jointOnePosition;
        }
        public double getJointTwoPosition() {
            return jointTwoPosition;
        }
    }
}
