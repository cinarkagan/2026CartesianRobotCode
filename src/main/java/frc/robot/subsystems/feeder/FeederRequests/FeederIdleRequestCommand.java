package frc.robot.subsystems.feeder.FeederRequests;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.AllStates;
import frc.robot.utils.AllStates.FeederStates;
import frc.robot.utils.AllStates.ShooterStates;

public class FeederIdleRequestCommand extends Command {
    private Feeder m_feeder;

    public FeederIdleRequestCommand(Feeder subsystem) {
        m_feeder = subsystem;
    }

    @Override
    public void initialize() {
        m_feeder.requestState(FeederStates.IDLE);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}