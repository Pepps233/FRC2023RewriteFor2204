package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.ArmConfiguration;
import frc.robot.lib.HomingNEOSparkMax;
import frc.robot.subsystems.ArmSubsystem;

public class TeleopPositionArm extends CommandBase {
    public TeleopPositionArm(ArmSubsystem armSubsystem) {
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        final ArmConfiguration.Positions positions = ArmSubsystem.selectedPosition;
        // updates the current positions name, if null, put "None"
        SmartDashboard.putString("Current position: ", positions == null ? "none" : positions.name());
        // execute only if current position is not null
        if (positions != null) {
            ArmSubsystem.firstJoint.setPosition(positions.getJointOnePosition() + ArmSubsystem.firstJointOffset);
            ArmSubsystem.secondJoint.setPosition(positions.getJointTwoPosition() + ArmSubsystem.secondJointOffset);
        }
    }

    // stop motors if command is interrupted
    @Override
    public void end(boolean interrupted) {
        HomingNEOSparkMax.stop();
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
