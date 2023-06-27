package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.ArmConfiguration;
import frc.robot.subsystems.ArmSubsystem;

public class SubOffsetJointTwo extends CommandBase {
    public SubOffsetJointTwo(ArmSubsystem armSubsystem) {
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        ArmSubsystem.secondJointOffset -= ArmConfiguration.OFFSET_AMOUNT;
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
