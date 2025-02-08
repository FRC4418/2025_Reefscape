// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ModeController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ToggleCommand extends Command {
  private Command command1;
  private Command command2;
  private Command mainCommand;
  private ModeController mode;
  private boolean hasRan = false;
  /** Creates a new ToggleCommand. */
  public ToggleCommand(Command command1, Command command2, ModeController mode) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.command1 = command1;
    this.command2 = command2;
    this.mode = mode;
    addRequirements(mode);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mainCommand = mode.isInCoralMode() ? command1 : command2;
    mainCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (hasRan) return;
    // command.schedule();
    // hasRan = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mainCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mainCommand.isFinished();
  }
}
