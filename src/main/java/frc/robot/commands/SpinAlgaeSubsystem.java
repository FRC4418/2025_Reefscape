// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.google.errorprone.annotations.ThreadSafe;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinAlgaeSubsystem extends Command {
  private final AlgaeSubsystem m_algeeSubsystem;
  private double intakeSpeed;
  private double shooterSpeed;
  /** Creates a new SpinAlgeeSubsystem. */
  public SpinAlgaeSubsystem(AlgaeSubsystem algeeSubsystem, double intakeSpeed, double shooterSpeed) {
    this.m_algeeSubsystem = algeeSubsystem;
    this.intakeSpeed = intakeSpeed;
    this.shooterSpeed = shooterSpeed;
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
    m_algeeSubsystem.spinShooters(intakeSpeed);
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
