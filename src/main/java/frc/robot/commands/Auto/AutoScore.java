// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Coral.SetCoralIntakePercentSpeed;
import frc.robot.commands.Coral.SetCoralPosition;
import frc.robot.subsystems.RobotStateController;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;
import frc.robot.subsystems.Manipulators.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScore extends Command {
  private DriveSubsystem m_robotDrive;
  private CoralSubsystem m_coralSubsystem;
  private RobotStateController m_robotStateController;
  Command command;
  /** Creates a new AutoScore. */
  public AutoScore(DriveSubsystem driveSubsystem, CoralSubsystem coralSubsystem, RobotStateController robotStateController) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(robotStateController, driveSubsystem, coralSubsystem);
    this.m_robotDrive = driveSubsystem;
    this.m_robotStateController = robotStateController;
    this.m_coralSubsystem = coralSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Command setPos = new SetCoralPosition(m_coralSubsystem, m_robotStateController.getElevatorScorePos(), m_robotStateController.getWristScorePos());

    Command driveToTarget = AutoBuilder.followPath(getTargetPath());

    Command driveToScore = AutoBuilder.followPath(getScorePath()).raceWith(setPos);
    
    Command score = new SetCoralIntakePercentSpeed(m_coralSubsystem, -1).raceWith(new WaitCommand(0.2)).raceWith(setPos);

    // command = driveToTarget.andThen(driveToScore).andThen(score);
    command = driveToTarget;
    // command.initialize();
    // command.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    command.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    command.end(interrupted);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return command.isFinished();
  }
  
  public PathPlannerPath getTargetPath(){
    var poses = new ArrayList<Pose2d>();

    poses.add(m_robotDrive.getPose());
    poses.add(m_robotStateController.getTargetPose());

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

    PathConstraints constraints = new PathConstraints(2, 1.5, 2 * Math.PI, 4 * Math.PI);

    return new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0, m_robotStateController.getTargetPose().getRotation()));

  }

  public PathPlannerPath getScorePath(){
    var poses = new ArrayList<Pose2d>();

    poses.add(m_robotStateController.getTargetPose());
    poses.add(m_robotStateController.getTargetPose().transformBy(m_robotStateController.getScoreTransform()));

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

    PathConstraints constraints = new PathConstraints(1, .5, 2 * Math.PI, 4 * Math.PI);

    return new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0, m_robotStateController.getTargetPose().getRotation()));
  }
}
