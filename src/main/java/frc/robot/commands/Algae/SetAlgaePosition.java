// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Algae;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.Manipulators.AlgaeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetAlgaePosition extends Command {
  /** Creates a new SetAlgaeElevatorPosition. */
  private AlgaeSubsystem m_algeeSubsystem;
  private double elevatorPos;
  private double wristPose;
  private PIDController elevatorPIDController = new PIDController(PIDConstants.kElevatorP, PIDConstants.kElevatorI, PIDConstants.kElevatorD);
  private PIDController wristPIDController = new PIDController(PIDConstants.kAlgaeWristP, PIDConstants.kAlgaeWristI, PIDConstants.kAlgaeWristD);
  public SetAlgaePosition(AlgaeSubsystem algaeSubsystem, double elevatorPos, double wristPose) {
    this.elevatorPos = elevatorPos;
    this.wristPose = wristPose;
    m_algeeSubsystem = algaeSubsystem;
    addRequirements(algaeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elevatorPIDvalue = elevatorPIDController.calculate(m_algeeSubsystem.getElevatorPos(), elevatorPos);
    double wristPIDValue = wristPIDController.calculate(m_algeeSubsystem.getWristPos(), wristPose);
    double wristStall = Math.cos(m_algeeSubsystem.getWristPos()) * PIDConstants.kAlgaeWristrStallMulti;

    m_algeeSubsystem.setElevatorPercentOutput(elevatorPIDvalue + PIDConstants.kAlgaeElevatorStall);
    m_algeeSubsystem.setWristPercentOutput(wristPIDValue + wristStall);
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
