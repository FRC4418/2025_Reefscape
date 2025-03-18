// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Manipulators.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetCoralIntakePercentSpeed extends Command {
  /** Creates a new SpinCoralSubsystem. */
  private CoralSubsystem m_coralSubsystem;
  private double speed;
  public SetCoralIntakePercentSpeed(CoralSubsystem coralSubsystem, double speed) {
    this.speed = speed;
    m_coralSubsystem = coralSubsystem;
    // addRequirements(coralSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_coralSubsystem.setIntakePercentOutput(speed);
  }

  // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {
  //   if(speed < -.1) return;
  //   m_coralSubsystem.setHasCoral(false);
  // }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
