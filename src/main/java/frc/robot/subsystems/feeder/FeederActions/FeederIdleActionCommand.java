package frc.robot.subsystems.feeder.FeederActions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.AllStates;

public class FeederIdleActionCommand extends Command {
    private Feeder m_feeder;

    public FeederIdleActionCommand(Feeder subsystem) {
        m_feeder = subsystem;
        addRequirements(m_feeder);
    }

    @Override
    public void initialize() {
        m_feeder.feederIdle();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}