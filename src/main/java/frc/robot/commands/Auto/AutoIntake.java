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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Coral.IntakeUntillGood;
import frc.robot.commands.Coral.SetCoralPosition;
import frc.robot.subsystems.RobotStateController;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;
import frc.robot.subsystems.Manipulators.CoralSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoIntake extends Command {
  private DriveSubsystem m_robotDrive;
  private RobotStateController m_robotStateController;
  private CoralSubsystem m_coralSubsystem;
  private Command command;
  /** Creates a new AutoIntake. */
  public AutoIntake(DriveSubsystem driveSubsystem , RobotStateController robotStateController, CoralSubsystem coralSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    m_robotDrive = driveSubsystem;
    m_robotStateController = robotStateController;
    m_coralSubsystem = coralSubsystem;
    addRequirements(driveSubsystem, robotStateController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    
    var poses = new ArrayList<Pose2d>();

    poses.add(m_robotDrive.getPose());
    poses.add(m_robotStateController.getIntakPos());

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

    PathConstraints constraints = new PathConstraints(1, 1, 2 * Math.PI, 4 * Math.PI);

    var path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0, m_robotStateController.getTargetPose().getRotation()));

    // Command intake = new SetCoralPosition(m_coralSubsystem, 70, 0).raceWith(new IntakeUntillGood(m_coralSubsystem, 1));


    command = AutoBuilder.followPath(path);//.andThen(intake);
    command.initialize();
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
}
