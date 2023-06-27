package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.ArmConfiguration;
import frc.robot.lib.HomingNEOSparkMax;

import javax.management.ConstructorParameters;

/*
Workflow: 1. PIDFBuilder(class for creating a pid controller object along with getter methods)
          2. Created ArmConfiguration class for spark id's, two joints' configurations, and new enum class "Positions"
             to store fixed positioning data for different objects on the field (ex. first node, second node positions)
          3. Created HomingConfiguration class and its constructor method to receive the information from the first
             joint and second joint's instantiation.
          4. Created HomingNEOSparkMax class and its constructor method to receive object creation with device spark
             id and object of type HomingConfiguration. The constructor method instantiates a new CANSparkMax object
             with device id and motor type. The new object is then used to set up the pid constants, setting the output
             range, setting the idle mode, and instantiating a new DigitalInput object and slewRateLimiter object.
          5. Created ArmSubsystem(this) class to instantiate the two joints with constructor HomingNEOSparkMax
             and creating three setter methods that sets the desired position of the arm. -> The target position,
             the forward cycle position and backward cycle position.
          6. Created class TeleopPositionArm and its constructor method that accepts an ArmSubsystem object used
             for addRequirements. The class has an execute method updates the robot's arm position status and setting
             the robot's arm positions if current arm position is not null.
          7. Inside RobotContainer, a new armSubsystem object of type ArmSubsystem is created, and set to default
             command by calling the constructor method of TeleopPositionArm and passing in an object of ArmSubsystem.
             Then,
 */
public class ArmSubsystem extends SubsystemBase {
    public static HomingNEOSparkMax firstJoint = new HomingNEOSparkMax(ArmConfiguration.FIRST_JOINT_SPARK_ID,
                                                                ArmConfiguration.FIRST_JOINT_CONFIG);
    public static HomingNEOSparkMax secondJoint = new HomingNEOSparkMax(ArmConfiguration.SECOND_JOINT_SPARK_ID,
                                                                 ArmConfiguration.SECOND_JOINT_CONFIG);
    public static double firstJointOffset = 0.0;
    public static double secondJointOffset = 0.0;
    public static ArmConfiguration.Positions selectedPosition = null;

    // this method sets target position from both joints
    public void setTargetPosition(ArmConfiguration.Positions position) {
        // only set position if position passed in is not null
        if (position != null) {
            selectedPosition = position;
            System.out.println("Selected position set to: " + position);
        }
    }
    // on button press -> cycles through positions of joints in a forward cycle
    public void setNextPosition() {
        // if current position is not null
        if (selectedPosition != null) {
            switch(selectedPosition) {
                case FLOOR_TIPPED:
                    selectedPosition = ArmConfiguration.Positions.FLOOR;
                    break;
                case FLOOR:
                    selectedPosition = ArmConfiguration.Positions.HOME;
                    break;
                case HOME:
                    selectedPosition = ArmConfiguration.Positions.FIRST_NODE;
                    break;
                case FIRST_NODE:
                    selectedPosition = ArmConfiguration.Positions.SECOND_NODE;
                    break;
                case SECOND_NODE:
                    selectedPosition = ArmConfiguration.Positions.THIRD_NODE;
                    break;
                case THIRD_NODE:
                    // do nothing, already at last position in cycle
                    break;
                default:
                    selectedPosition = ArmConfiguration.Positions.HOME;
            }
        }
    }
    // on button press -> cycles through positions of joints in a backwards cycle
    public void setPreviousPosition() {
        // if current position is not null
        if (selectedPosition != null) {
            switch(selectedPosition) {
                case FLOOR_TIPPED:
                    // do nothing, already at last position in cycle
                    break;
                case FLOOR:
                    selectedPosition = ArmConfiguration.Positions.FLOOR_TIPPED;
                    break;
                case HOME:
                    selectedPosition = ArmConfiguration.Positions.FLOOR;
                    break;
                case FIRST_NODE:
                    selectedPosition = ArmConfiguration.Positions.HOME;
                    break;
                case SECOND_NODE:
                    selectedPosition = ArmConfiguration.Positions.FIRST_NODE;
                    break;
                case THIRD_NODE:
                    selectedPosition = ArmConfiguration.Positions.SECOND_NODE;
                    break;
                default:
                    selectedPosition = ArmConfiguration.Positions.HOME;
            }
        }
    }
    @Override
    public void periodic() {
        // monitoring arm status
        SmartDashboard.putBoolean("First joint homed: ", firstJoint.isAtHome());
        SmartDashboard.putBoolean("Second joint homed: ", secondJoint.isAtHome());
        SmartDashboard.putNumber("First joint position: ", firstJoint.getPosition());
        SmartDashboard.putNumber("Second joint position: ", secondJoint.getPosition());
    }
}
