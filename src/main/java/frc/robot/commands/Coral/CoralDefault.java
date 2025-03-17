// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulators.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralDefault extends Command {
  private CoralSubsystem m_coralSubsystem;

  private Command goDown;

  /** Creates a new CoralDefault. */
  public CoralDefault(CoralSubsystem coralSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(coralSubsystem);

    m_coralSubsystem = coralSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goDown = new SetCoralPosition(m_coralSubsystem, 0, 0.13);
    goDown.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_coralSubsystem.getElevatorPos() > 2){
      goDown.execute(); 
      m_coralSubsystem.setIntakePercentOutput(0);
    }else{
      m_coralSubsystem.setElevatorPercentOutput(0);
      m_coralSubsystem.setWristPercentOutput(0);
    }
    m_coralSubsystem.setIntakePercentOutput(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    goDown.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
