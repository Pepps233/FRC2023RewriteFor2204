package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class CloseClaw extends CommandBase {
    public CloseClaw(ClawSubsystem clawSubsystem) {
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        ClawSubsystem.close();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
