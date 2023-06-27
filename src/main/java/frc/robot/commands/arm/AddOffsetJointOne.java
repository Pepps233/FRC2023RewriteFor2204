package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.ArmConfiguration;
import frc.robot.subsystems.ArmSubsystem;

public class AddOffsetJointOne extends CommandBase {
    public AddOffsetJointOne(ArmSubsystem armSubsystem) {
        addRequirements(armSubsystem);
    }
    // initialize() will immediately be called when we create a new instance of this class and passing in the subsystem
    // as the requirement
    @Override
    public void initialize() {
        ArmSubsystem.firstJointOffset += ArmConfiguration.OFFSET_AMOUNT;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
