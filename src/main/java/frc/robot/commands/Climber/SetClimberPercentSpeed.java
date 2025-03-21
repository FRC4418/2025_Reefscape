// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulators.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetClimberPercentSpeed extends Command {
  private final ClimberSubsystem m_climberSubsystem;
  private double speed;
  /** Creates a new SetClimberPos. */
  public SetClimberPercentSpeed(ClimberSubsystem subsystem, double speed) {

    m_climberSubsystem = subsystem;

    this.speed = speed;

    addRequirements(m_climberSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(m_climberSubsystem.canRun == false) return;
    m_climberSubsystem.setPercentSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
