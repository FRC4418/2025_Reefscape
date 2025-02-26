// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Algae;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulators.AlgaeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetAlgaeIntakePercentSpeed extends Command {
  private final AlgaeSubsystem m_algeeSubsystem;
  private double intakeSpeed;
  /** Creates a new SpinAlgeeSubsystem. */
  public SetAlgaeIntakePercentSpeed(AlgaeSubsystem algeeSubsystem, double intakeSpeed) {
    this.m_algeeSubsystem = algeeSubsystem;
    this.intakeSpeed = intakeSpeed;
    addRequirements(m_algeeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_algeeSubsystem.spinIntake(intakeSpeed);
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
