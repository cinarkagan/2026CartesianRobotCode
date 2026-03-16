package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.fasterxml.jackson.databind.annotation.EnumNaming;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.AllStates;
import frc.robot.utils.AllStates.ShooterStates;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;

public class ShooterSubsystem extends Shooter {
    SparkMaxConfig config = new SparkMaxConfig();
    private final SparkMax shooter1 = new SparkMax(ShooterConstants.shooter1_ID,MotorType.kBrushless);
    SparkClosedLoopController shooter1PID = shooter1.getClosedLoopController();
    private final TalonFX shooter2 = new TalonFX(ShooterConstants.shooter2_ID, ShooterConstants.canbus);
    private final TalonFX feeder = new TalonFX(ShooterConstants.feeder_ID, ShooterConstants.canbus);

    public double goalRPM = ShooterConstants.IDLE_RPM;
    public double feederRPM = 0;
    private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

    public TalonFXConfiguration configs_shooter = new TalonFXConfiguration();
    public TalonFXConfiguration configs_feeder = new TalonFXConfiguration();
    public AllStates.ShooterStates currentState = ShooterStates.IDLE;
    public AllStates.ShooterStates requestedState = ShooterStates.IDLE;
    public AllStates.ShooterStates lastState = ShooterStates.IDLE;

    public ShooterSubsystem() {
            /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        configs_shooter.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
        configs_shooter.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        configs_shooter.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
        configs_shooter.Slot0.kI = 0; // No output for integrated error
        configs_shooter.Slot0.kD = 0; // No output for error derivative
        // Peak output of 8 volts
        configs_shooter.Voltage.withPeakForwardVoltage(Volts.of(8))
            .withPeakReverseVoltage(Volts.of(-8));
        shooter2.getConfigurator().apply(configs_shooter);

            /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
        configs_feeder.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
        configs_feeder.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        configs_feeder.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
        configs_feeder.Slot0.kI = 0; // No output for integrated error
        configs_feeder.Slot0.kD = 0; // No output for error derivative
        // Peak output of 8 volts
        configs_feeder.Voltage.withPeakForwardVoltage(Volts.of(8))
            .withPeakReverseVoltage(Volts.of(-8));

        feeder.getConfigurator().apply(configs_feeder);

        //configure shooter motor 1
        config.inverted(ShooterConstants.shooter1_reversed).idleMode(IdleMode.kBrake);
        config.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.11, 0.0, 0.0);
        shooter1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        stateMachine();
        rpmControl();
    }

    public void rpmControl() {
        double motorRPM = goalRPM*ShooterConstants.flywheelGearRatio;
        shooter1.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        shooter2.setControl(m_velocityVoltage.withVelocity(motorRPM/60));
        feeder.setControl(m_velocityVoltage.withVelocity(feederRPM/60));
    }

    public boolean isAtRPM(){
        boolean isShooter1AtRPM = ((getVelocityShooter1()>(goalRPM-ShooterConstants.rpmTol))&&(getVelocityShooter1()<(goalRPM+ShooterConstants.rpmTol)));
        if (isShooter1AtRPM){
            if ((shooter2.getVelocity().getValueAsDouble()*60>(goalRPM-ShooterConstants.rpmTol))&&((shooter2.getVelocity().getValueAsDouble())*60<(goalRPM+ShooterConstants.rpmTol))){
                return true;
            }
        }
        return false;
    }

    public void stateMachine() {
        if (currentState == requestedState){
                switch(currentState) {
                    case IDLE:
                        goalRPM = ShooterConstants.IDLE_RPM;
                        break;
                    case KILL:
                        goalRPM = 0;
                        break;
                    case LOW_POWER:
                        goalRPM = ShooterConstants.LOW_POWER_RPM;
                        break;
                    case SHOOT:
                        goalRPM = 0;
                        break;
                    case REVERSE:
                        goalRPM = ShooterConstants.REVERSE_RPM;
                        break;

                }
        } else {
            lastState = currentState;
            currentState = requestedState;
            stateMachine();
        }
    }
    public double getRPMGoal() {
        return goalRPM;
    }
    public double getRPMHood() {
        return (shooter2.getVelocity().getValueAsDouble()*60/ShooterConstants.hoodGearReduction);
    }
    public double getRPMFlywheel() {
        return (getVelocityShooter1()/ShooterConstants.flywheelGearRatio);
    }

    @Override
    public void requestState(ShooterStates state) {
        requestedState = state;
    }

    public double getVelocityShooter1(){
        return shooter1.getEncoder().getVelocity();
    }
    
    @Override
    public void shooterIdle() {
        goalRPM = ShooterConstants.IDLE_RPM;
    }

    @Override
    public void shooterKill() {
        goalRPM = 0;
    }

    @Override
    public void shooterLowPower() {
        goalRPM = ShooterConstants.LOW_POWER_RPM;
    }

    @Override
    public void shooterShoot() { /*** TODO: Add calculate RPM to this */
        goalRPM = 0;
    }

    @Override
    public void shooterReverse() {
        goalRPM = ShooterConstants.REVERSE_RPM;
    }
}
