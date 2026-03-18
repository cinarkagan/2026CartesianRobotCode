package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.FeederConstants;
import frc.robot.subsystems.feeder.FeederActions.*;
import frc.robot.utils.AllStates;
import frc.robot.utils.AllStates.FeederStates;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;

public class FeederSubsystem extends Feeder {

    private final SparkMax feeder1 = new SparkMax(FeederConstants.feeder1_ID, MotorType.kBrushless);
    private final SparkMax feeder2 = new SparkMax(FeederConstants.feeder2_ID, MotorType.kBrushless);

    SparkMaxConfig configFeeder1 = new SparkMaxConfig();
    SparkMaxConfig configFeeder2 = new SparkMaxConfig();

    public double goalRPM = FeederConstants.IDLE_RPM;
    public AllStates.FeederStates currentState   = FeederStates.IDLE;
    public AllStates.FeederStates requestedState = FeederStates.IDLE;
    public AllStates.FeederStates lastState      = FeederStates.IDLE;

    private FeederIdleActionCommand    feederIdleActionCommand;
    private FeederKillActionCommand    feederKillActionCommand;
    private FeederFeedActionCommand    feederFeedActionCommand;
    private FeederReverseActionCommand feederReverseActionCommand;
    private FeederTestActionCommand    feederTestActionCommand;

    public FeederSubsystem() {
        // Feeder 1
        configFeeder1.inverted(FeederConstants.feeder1_reversed).idleMode(IdleMode.kBrake);
        configFeeder1.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
        configFeeder1.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(FeederConstants.kP, FeederConstants.kI, FeederConstants.kD);
        feeder1.configure(configFeeder1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Feeder 2
        configFeeder2.inverted(FeederConstants.feeder2_reversed).idleMode(IdleMode.kBrake);
        configFeeder2.encoder.positionConversionFactor(1000).velocityConversionFactor(1000);
        configFeeder2.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(FeederConstants.kP, FeederConstants.kI, FeederConstants.kD);
        feeder2.configure(configFeeder2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Action Commands
        feederIdleActionCommand    = new FeederIdleActionCommand(this);
        feederKillActionCommand    = new FeederKillActionCommand(this);
        feederFeedActionCommand    = new FeederFeedActionCommand(this);
        feederReverseActionCommand = new FeederReverseActionCommand(this);
        feederTestActionCommand    = new FeederTestActionCommand(this);
    }

    @Override
    public void periodic() {
        stateMachine();
        rpmControl();
        if (SmartDashboard.getBoolean("FeederTelemetry", false) || SmartDashboard.getBoolean("AllTelemetry", false)) {
            telemetrize();
        }
    }

    public void rpmControl() {
        double motorRPM = goalRPM * FeederConstants.feederGearRatio;
        feeder1.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        feeder2.getClosedLoopController().setSetpoint(motorRPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    @Override
    public boolean isAtRPM() {
        return isMotorAtRPM(getVelocityFeeder1())
            && isMotorAtRPM(getVelocityFeeder2());
    }

    private boolean isMotorAtRPM(double velocity) {
        return velocity > (goalRPM - FeederConstants.rpmTol)
            && velocity < (goalRPM + FeederConstants.rpmTol);
    }

    public void stateMachine() {
        if (currentState == requestedState) {
            switch (currentState) {
                case IDLE:
                    if (!feederIdleActionCommand.isScheduled())
                        CommandScheduler.getInstance().schedule(feederIdleActionCommand);
                    break;
                case KILL:
                    if (!feederKillActionCommand.isScheduled())
                        CommandScheduler.getInstance().schedule(feederKillActionCommand);
                    break;
                case FEED:
                    if (!feederFeedActionCommand.isScheduled())
                        CommandScheduler.getInstance().schedule(feederFeedActionCommand);
                    break;
                case REVERSE:
                    if (!feederReverseActionCommand.isScheduled())
                        CommandScheduler.getInstance().schedule(feederReverseActionCommand);
                    break;
                case TEST:
                    if (!feederTestActionCommand.isScheduled())
                        CommandScheduler.getInstance().schedule(feederTestActionCommand);
                    break;
                default:
                    requestState(FeederStates.IDLE);
                    break;
            }
        } else {
            lastState = currentState;
            currentState = requestedState;
            stateMachine();
        }
    }

    @Override
    public void feederIdle() {
        goalRPM = FeederConstants.IDLE_RPM;
    }

    @Override
    public void feederKill() {
        goalRPM = 0;
    }

    @Override
    public void feederFeed() {
        goalRPM = FeederConstants.FEED_RPM;
    }

    @Override
    public void feederReverse() {
        goalRPM = FeederConstants.REVERSE_RPM;
    }

    @Override
    public void feederTest() {
        goalRPM = SmartDashboard.getNumber("TestFeederRPM", FeederConstants.IDLE_RPM);
    }

    @Override
    public void requestState(FeederStates state) {
        requestedState = state;
    }
    
    public void telemetrize() {
        SmartDashboard.putNumber("Feeder/RPM1", getVelocityFeeder1());
        SmartDashboard.putNumber("Feeder/RPM2", getVelocityFeeder2());
        SmartDashboard.putNumber("Feeder/RPM Average", getAverageRPM());
        SmartDashboard.putNumber("Feeder/RPM Goal", getRPMGoal());
        SmartDashboard.putBoolean("Feeder/Is At RPM", isAtRPM());
        SmartDashboard.putString("Feeder/Current State", currentState.toString());
        SmartDashboard.putString("Feeder/Requested State", requestedState.toString());
    }

    public double getRPMGoal()      { return goalRPM; }
    public double getRPMFeeder()    { return getAverageRPM() / FeederConstants.feederGearRatio; }

    public double getVelocityFeeder1() { return feeder1.getEncoder().getVelocity(); }
    public double getVelocityFeeder2() { return feeder2.getEncoder().getVelocity(); }

    public double getAverageRPM() {
        return (getVelocityFeeder1() + getVelocityFeeder2()) / 2.0;
    }
}