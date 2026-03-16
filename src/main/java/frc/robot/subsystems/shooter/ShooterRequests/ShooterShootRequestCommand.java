package frc.robot.subsystems.shooter.ShooterRequests;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.AllStates;

public class ShooterShootRequestCommand extends Command {
  // The subsystem the command runs on
  private Shooter m_shooter;

  public ShooterShootRequestCommand(Shooter subsystem) {
    m_shooter = subsystem;
  }

  @Override
  public void initialize() {
    m_shooter.requestState(AllStates.ShooterStates.SHOOT);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}