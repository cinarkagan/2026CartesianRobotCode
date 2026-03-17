package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterActions.ShooterIdleActionCommand;
import frc.robot.subsystems.shooter.ShooterActions.ShooterKillActionCommand;
import frc.robot.subsystems.shooter.ShooterActions.ShooterLowPowerActionCommand;
import frc.robot.subsystems.shooter.ShooterActions.ShooterReverseActionCommand;
import frc.robot.subsystems.shooter.ShooterActions.ShooterShootActionCommand;
import frc.robot.subsystems.shooter.ShooterUtils.ShooterCalculator;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.utils.AllStates;
import frc.robot.utils.AllStates.ShooterStates;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;

public class ShooterSimulation extends Shooter {

    public double goalRPM = ShooterConstants.IDLE_RPM;
    public AllStates.ShooterStates currentState   = ShooterStates.IDLE;
    public AllStates.ShooterStates requestedState = ShooterStates.IDLE;
    public AllStates.ShooterStates lastState      = ShooterStates.IDLE;

    // Action Commands
    private ShooterIdleActionCommand shooterIdleActionCommand;
    private ShooterKillActionCommand shooterKillActionCommand;
    private ShooterLowPowerActionCommand shooterLowPowerActionCommand;
    private ShooterShootActionCommand shooterShootActionCommand;
    private ShooterReverseActionCommand shooterReverseActionCommand;

    private final ShooterCalculator shooterCalc;

    //TODO: PID Configleri constantsa çek(claude yapar)

    public ShooterSimulation(CommandSwerveDrivetrain commandSwerveDrivetrain) {
        shooterCalc = new ShooterCalculator(commandSwerveDrivetrain);
        // Action Commands
        shooterIdleActionCommand     = new ShooterIdleActionCommand(this);
        shooterKillActionCommand     = new ShooterKillActionCommand(this);
        shooterLowPowerActionCommand = new ShooterLowPowerActionCommand(this);
        shooterShootActionCommand    = new ShooterShootActionCommand(this);
        shooterReverseActionCommand  = new ShooterReverseActionCommand(this);
    }

    @Override
    public void periodic() {
        stateMachine();
        rpmControl();
        if (SmartDashboard.getBoolean("ShooterTelemetry", true)||SmartDashboard.getBoolean("AllTelemetry", false)) {telemetrize();}
    }

    public void telemetrize() {
        SmartDashboard.putNumber("Shooter/RPM1", getVelocityShooter1());
        SmartDashboard.putNumber("Shooter/RPM2", getVelocityShooter2());
        SmartDashboard.putNumber("Shooter/RPM3", getVelocityShooter3());
        SmartDashboard.putNumber("Shooter/RPM4", getVelocityShooter4());
        SmartDashboard.putNumber("Shooter/RPM5", getVelocityShooter5());
        SmartDashboard.putNumber("Shooter/RPM6", getVelocityShooter6());
        SmartDashboard.putNumber("Shooter/RPM Average", getAverageRPM());
        SmartDashboard.putNumber("Shooter/RPM Goal", getRPMGoal());
        SmartDashboard.putBoolean("Shooter/Is At RPM", isAtRPM());
        SmartDashboard.putString("Shooter/Current State", currentState.toString());
        SmartDashboard.putString("Shooter/Requested State", requestedState.toString());
    }

    // ── RPM Control ───────────────────────────────────────────────────────────
    public void rpmControl() {    }

    @Override
    public boolean isAtRPM() {
        return isMotorAtRPM(getVelocityShooter1())
            && isMotorAtRPM(getVelocityShooter2())
            && isMotorAtRPM(getVelocityShooter3())
            && isMotorAtRPM(getVelocityShooter4())
            && isMotorAtRPM(getVelocityShooter5())
            && isMotorAtRPM(getVelocityShooter6());
    }

    private boolean isMotorAtRPM(double velocity) {
        return velocity > (goalRPM - ShooterConstants.rpmTol)
            && velocity < (goalRPM + ShooterConstants.rpmTol);
    }

public void stateMachine() {
    if (currentState == requestedState) {
        switch (currentState) {
            case IDLE:
                if (!shooterIdleActionCommand.isScheduled())
                    CommandScheduler.getInstance().schedule(shooterIdleActionCommand);
                break;
            case KILL:
                if (!shooterKillActionCommand.isScheduled())
                    CommandScheduler.getInstance().schedule(shooterKillActionCommand);
                break;
            case LOW_POWER:
                if (!shooterLowPowerActionCommand.isScheduled())
                    CommandScheduler.getInstance().schedule(shooterLowPowerActionCommand);
                break;
            case SHOOT:
                if (!shooterShootActionCommand.isScheduled())
                    CommandScheduler.getInstance().schedule(shooterShootActionCommand);
                break;
            case REVERSE:
                if (!shooterReverseActionCommand.isScheduled())
                    CommandScheduler.getInstance().schedule(shooterReverseActionCommand);
                break;
        }
    } else {
        lastState = currentState;
        currentState = requestedState;
        stateMachine();
    }
}

    @Override
    public void shooterIdle() {
        goalRPM = shooterCalc.calculateRestFlywheelSpeedFromCurrentPose();
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
    public void shooterShoot() {
        goalRPM = shooterCalc.calculateFlywheelRPMFromCurrentPose();
    }

    @Override
    public void shooterReverse() {
        goalRPM = ShooterConstants.REVERSE_RPM;
    }

    @Override
    public void requestState(ShooterStates state) {
        requestedState = state;
    }

    public double getRPMGoal()     { return goalRPM; }

    public double getRPMFlywheel() { return getAverageRPM() / ShooterConstants.flywheelGearRatio; }

    public double getVelocityShooter1() { return goalRPM;}
    public double getVelocityShooter2() { return goalRPM; }
    public double getVelocityShooter3() { return goalRPM; }
    public double getVelocityShooter4() { return goalRPM; }
    public double getVelocityShooter5() { return goalRPM; }
    public double getVelocityShooter6() { return goalRPM; }
    public double getAverageRPM() {
        return (getVelocityShooter1()
            + getVelocityShooter2()
            + getVelocityShooter3()
            + getVelocityShooter4()
            + getVelocityShooter5()
            + getVelocityShooter6()) / 6.0;
    }
}