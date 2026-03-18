package frc.robot.subsystems.feeder.FeederActions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.feeder.Feeder;

public class FeederReverseActionCommand extends Command {
    private Feeder m_feeder;

    public FeederReverseActionCommand(Feeder subsystem) {
        m_feeder = subsystem;
        addRequirements(m_feeder);
    }

    @Override
    public void initialize() {
        m_feeder.feederReverse();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}