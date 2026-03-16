package frc.robot.subsystems.shooter.ShooterActions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.AllStates;

public class ShooterIdleActionCommand extends Command {
    private Shooter m_shooter;

    public ShooterIdleActionCommand(Shooter subsystem) {
        m_shooter = subsystem;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.shooterIdle();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}