// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Coral;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.Manipulators.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetCoralPosition extends Command {
  /** Creates a new SetCoralElevatorPosition. */
  private CoralSubsystem m_coralSubsystem;
  private double elevatorPos;
  private double wristPose;
  private PIDController elevatorPIDController = new PIDController(PIDConstants.kElevatorP, PIDConstants.kElevatorI, PIDConstants.kElevatorD);
  private PIDController wristPIDController = new PIDController(PIDConstants.kCoralWristP, PIDConstants.kCoralWristI, PIDConstants.kCoralWristD);
  public SetCoralPosition(CoralSubsystem coralSubsystem, double elevatorPos, double wristPose) {
    this.elevatorPos = elevatorPos;
    this.wristPose = wristPose;
    m_coralSubsystem = coralSubsystem;
    addRequirements(coralSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double elevatorPIDvalue = elevatorPIDController.calculate(m_coralSubsystem.getElevatorPos(), elevatorPos);
    double wristPIDValue = wristPIDController.calculate(m_coralSubsystem.getWristPos(), wristPose);
    double wristStall = Math.cos(2*Math.PI*(m_coralSubsystem.getWristPos()-0.1)) * PIDConstants.kCoralWristrStallMulti;

    m_coralSubsystem.setWristPercentOutput(wristPIDValue + wristStall);


    // SmartDashboard.putNumber("command wrist pose", m_coralSubsystem.getWristPos());
    // SmartDashboard.putNumber("command wrist pid out", wristPIDValue);

    if(elevatorPos - 2 < m_coralSubsystem.getElevatorPos()  || m_coralSubsystem.getElevatorPos() < elevatorPIDvalue + 2){
      m_coralSubsystem.setElevatorPercentOutput(elevatorPIDvalue);// + PIDConstants.kCoralElevatorStall);
    }else{
      m_coralSubsystem.setPosFancy(elevatorPos);
    }
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
