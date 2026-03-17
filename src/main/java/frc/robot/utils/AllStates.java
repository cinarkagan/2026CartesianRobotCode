package frc.robot.utils;

public class AllStates {
    public static enum ShooterStates {
        IDLE,
        LOW_POWER,
        KILL,
        SHOOT,
        REVERSE
    }
    public static enum FeederStates {
        IDLE,
        KILL,
        FEED,
        REVERSE
    }
    public static enum MachineStates {
        SHOOT,
        IDLE,
        INTAKE,
        INTAKE_CLOSE
    }
}
