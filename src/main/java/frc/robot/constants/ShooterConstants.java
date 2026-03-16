package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ShooterConstants {
    public static CANBus canbus = new CANBus("rio");

    public static int shooter1_ID = 61;
    public static boolean shooter1_reversed = false;
    public static int shooter2_ID = 24;
    public static boolean shooter2_reversed = false;
    public static int feeder_ID = 25;
    public static boolean feeder_reversed = false;


    public static double feederRPM = 2000;
    public static double IDLE_RPM = 1000;
    public static double LOW_POWER_RPM = 0;
    public static double REVERSE_RPM = -IDLE_RPM;
    public static double rpmTol = 100;
    
    public static double flywheelGearRatio = 1.5;
    public static double hoodGearReduction = 1.5;

}
