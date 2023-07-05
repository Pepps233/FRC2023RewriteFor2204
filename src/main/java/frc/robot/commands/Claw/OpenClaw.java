package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class OpenClaw extends CommandBase {
    public OpenClaw(ClawSubsystem clawSubsystem) {
        addRequirements(clawSubsystem);
    }
    @Override
    public void initialize() {
        ClawSubsystem.open();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
