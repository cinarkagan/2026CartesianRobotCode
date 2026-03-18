package frc.robot.subsystems.feeder.FeederActions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;

public class FeederKillActionCommand extends Command {
    private Feeder m_feeder;

    public FeederKillActionCommand(Feeder subsystem) {
        m_feeder = subsystem;
        addRequirements(m_feeder);
    }

    @Override
    public void initialize() {
        m_feeder.feederKill();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}