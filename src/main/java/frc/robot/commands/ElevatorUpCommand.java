package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Elevator;

public class ElevatorUpCommand extends Command {
    private final Elevator elevator;

    public ElevatorUpCommand(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(this.elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        elevator.setVelocity(0.5);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
