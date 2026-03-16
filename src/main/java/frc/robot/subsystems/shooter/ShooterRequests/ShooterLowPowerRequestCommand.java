package frc.robot.subsystems.shooter.ShooterRequests;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.AllStates;

public class ShooterLowPowerRequestCommand extends Command {
    private Shooter m_shooter;

    public ShooterLowPowerRequestCommand(Shooter subsystem) {
        m_shooter = subsystem;
    }

    @Override
    public void initialize() {
        m_shooter.requestState(AllStates.ShooterStates.LOW_POWER);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}