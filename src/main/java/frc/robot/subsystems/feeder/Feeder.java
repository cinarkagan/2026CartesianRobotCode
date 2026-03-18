package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.AllStates;

public abstract class Feeder extends SubsystemBase {
    public abstract void requestState(AllStates.FeederStates state);
    public abstract boolean isAtRPM();

    public abstract void feederIdle();
    public abstract void feederKill();
    public abstract void feederFeed();
    public abstract void feederReverse();
    public abstract void feederTest();

}