package frc.robot.subsystems.shooter;

import java.util.Random;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SimulationConstants;
import frc.robot.subsystems.shooter.ShooterUtils.BallKinematics;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.utils.AllStates;
import frc.robot.utils.AllStates.ShooterStates;
import frc.robot.utils.Container;
import frc.robot.utils.FuelSim;

public class ShooterSimulation extends Shooter {
    CommandSwerveDrivetrain drivetrain;
    
    public double goalRPM = ShooterConstants.IDLE_RPM;
    public double goalRPM2 = -ShooterConstants.IDLE_RPM;

    public AllStates.ShooterStates currentState = ShooterStates.IDLE;
    public AllStates.ShooterStates requestedState = ShooterStates.IDLE;
    public AllStates.ShooterStates lastState = ShooterStates.IDLE;

    public ShooterSimulation(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }
    public void launchFuel() {
        if (Container.fuelCount == 0) return;
        Container.decreaseFuel();
        Pose2d robot = drivetrain.getStateCopy().Pose;

        Translation3d initialPosition = new Translation3d(robot.getTranslation().getX(),robot.getTranslation().getY(),SimulationConstants.ShooterConstants.ShooterHeight);
        FuelSim.getInstance().spawnFuel(initialPosition, getLaunchVelocities(robot,goalRPM));//launchVel(vel, angle) with calc
    }
    public Command launchFuelCommand() {
        return new InstantCommand(()->{launchFuel();});
    }
    public Command launchFuelWithRandomnessCommand() {
        return new Command() {
            private long startTime;
            private int amountToShoot;
            private int ballsShot;
            private final long DELAY_MS = 50;

            @Override
            public void initialize() {
                if (Container.fuelCount > 0) {
                    startTime = System.currentTimeMillis();
                    
                    Random random = new Random();
                    double percentage = random.nextInt(100) / 100.0;
                    amountToShoot = (int) (Container.fuelCount * percentage) + 1;
                    
                    ballsShot = 0;
                } else {
                    amountToShoot = 0;
                }
            }

            @Override
            public void execute() {
                if (amountToShoot == 0) return;
                long elapsed = System.currentTimeMillis() - startTime;
                if (ballsShot < amountToShoot && elapsed > (ballsShot * DELAY_MS)) {
                    launchFuel();
                    ballsShot++;
                }
            }

            @Override
            public boolean isFinished() {
                return ballsShot >= amountToShoot;
            }

            @Override
            public void end(boolean interrupted) {
                Container.fuelCount = 0;
            }
        };
    }
    public Translation3d getLaunchVelocities(Pose2d robot,double rpm) {
        double heading = robot.getRotation().getRadians();
        double total = BallKinematics.totalBallVelocity(rpm);
        double y = BallKinematics.getVy(total);
        double x = BallKinematics.getVx(total, heading);
        double z = BallKinematics.getVz(total, heading);
        SmartDashboard.putNumber("Calc Y",y);
        SmartDashboard.putNumber("Calc X",x);
        SmartDashboard.putNumber("Calc Z",z);

        return new Translation3d(x,y,z);
    }

    @Override
    public void periodic() {
        stateMachine();
    }
    public boolean isAtRPM(){
        return true;
    }
    public double getRPM() { return goalRPM*1.5; } //gear ratio
    public double getRPMHood() { return goalRPM2*2; } //gear ratio
    public double getRPMFlywheel() { return goalRPM2; } //gear ratio

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
    @Override
    public void requestState(AllStates.ShooterStates state) {
        requestedState = state;
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
    public void shooterShoot() {
        goalRPM = 0;
    }

    @Override
    public void shooterReverse() {
        goalRPM = ShooterConstants.REVERSE_RPM;
    }
}