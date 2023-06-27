package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.ArmConfiguration;
import frc.robot.subsystems.ArmSubsystem;

public class SubOffsetJointOne extends CommandBase {
    public SubOffsetJointOne(ArmSubsystem armSubsystem) {
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        ArmSubsystem.firstJointOffset -= ArmConfiguration.OFFSET_AMOUNT;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
