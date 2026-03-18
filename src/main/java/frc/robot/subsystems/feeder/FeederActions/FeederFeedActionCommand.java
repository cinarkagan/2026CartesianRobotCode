package frc.robot.subsystems.feeder.FeederActions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;

public class FeederFeedActionCommand extends Command {
    private Feeder m_feeder;

    public FeederFeedActionCommand(Feeder subsystem) {
        m_feeder = subsystem;
        addRequirements(m_feeder);
    }

    @Override
    public void initialize() {
        m_feeder.feederFeed();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}