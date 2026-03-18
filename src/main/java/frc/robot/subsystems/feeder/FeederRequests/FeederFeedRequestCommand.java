package frc.robot.subsystems.feeder.FeederRequests;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.utils.AllStates.FeederStates;

public class FeederFeedRequestCommand extends Command {
    private Feeder m_feeder;

    public FeederFeedRequestCommand(Feeder subsystem) {
        m_feeder = subsystem;
    }

    @Override
    public void initialize() {
        m_feeder.requestState(FeederStates.FEED);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}