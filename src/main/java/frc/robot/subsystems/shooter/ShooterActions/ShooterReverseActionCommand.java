package frc.robot.subsystems.shooter.ShooterActions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterReverseActionCommand extends Command {
    private Shooter m_shooter;

    public ShooterReverseActionCommand(Shooter subsystem) {
        m_shooter = subsystem;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.shooterReverse();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
