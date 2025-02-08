// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignWithTag extends Command {
  private VisionSubsystem m_visionSubsystem;
  private DriveSubsystem m_robotDrive;
  public boolean foundTag = false;
  private Command path;

  /** Creates a new AlignWithTag. */
  public AlignWithTag(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
    addRequirements(visionSubsystem, driveSubsystem);
    m_robotDrive = driveSubsystem;
    m_visionSubsystem = visionSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose3d targetPose3d = m_visionSubsystem.targetPose3dRobotRelative();
    if (targetPose3d == null) return;
    initializePath(targetPose3d);
  }

  private void initializePath(Pose3d targetPose3d){
    
    Pose2d targetPose = new Pose2d(-targetPose3d.getZ() + 1,-targetPose3d.getX(), m_robotDrive.getPose().getRotation());

    Transform2d desiredTransform = new Transform2d(new Pose2d(), targetPose);
    Pose2d endPose = m_robotDrive.getPose().transformBy(desiredTransform);

    path = new DriveToPose(m_robotDrive, endPose);

    path.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    path.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return path.isFinished();
  }
}
