package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.AllStates;

public abstract class Shooter extends SubsystemBase {
    public abstract void requestState(AllStates.ShooterStates state);
    public abstract boolean isAtRPM();

    public abstract void shooterIdle();
    public abstract void shooterKill();
    public abstract void shooterLowPower();
    public abstract void shooterShoot();
    public abstract void shooterReverse();
    public abstract void shooterTest();
}