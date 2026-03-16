package frc.robot.subsystems.intake;

import java.util.Random;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.utils.Container;

public class IntakeSimulation implements Intake {
    private boolean isActive = false;
    public IntakeSimulation() {}
    
    public void intakeBall(double amout) {Container.fuelCount = Container.fuelCount + amout;}
    public Command intakeBallWithRandomness(double amount) {
        return new InstantCommand(() -> {
        if (isActive) {
            Random random = new Random();
            double percentage = random.nextDouble(100); 
            int amounttrue = (int) (amount * (percentage/100));
            intakeBall(amounttrue);
        }});
    }
    @Override
    public void activateIntake() {isActive = true;}
    @Override
    public void deactivateIntake() {isActive = false;}
    @Override
    public boolean getActive() {return isActive;}
    @Override
    public void toggleIntake() {isActive = !isActive;}
    public Command activateIntakeCommand() {
        return new InstantCommand(this::activateIntake);
    }
    public Command deactivateIntakeCommand() {
        return new InstantCommand(this::deactivateIntake);
    }
}
