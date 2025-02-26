// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.events.CancelCommandEvent;
import com.pathplanner.lib.events.EventScheduler;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {
  private Pose2d endPose;
  private DriveSubsystem m_driveSubsystem;
  private Command driveCommand;
  /** Creates a new DriveToPose. */
  public DriveToPose(DriveSubsystem driveSubsystem,Pose2d pose2d) {
    endPose = pose2d;
    m_driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var poses = new ArrayList<Pose2d>();

    poses.add(m_driveSubsystem.getPose());
    poses.add(endPose);

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

    PathConstraints constraints = new PathConstraints(2, 1.5, 2 * Math.PI, 4 * Math.PI);

    var path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0, endPose.getRotation()));

    driveCommand = AutoBuilder.followPath(path);

    driveCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return driveCommand.isFinished();
  }
}
