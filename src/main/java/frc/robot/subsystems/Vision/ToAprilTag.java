// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;

public class ToAprilTag extends SubsystemBase {
  /** Creates a new ToAprilTag. */
private VisionSubsystem m_vision;
  private DriveSubsystem m_robotDrive;
  private Command drive;
  private PathPlannerPath path;
  /** Creates a new ToAprilTag. */
  public ToAprilTag(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
    m_vision = visionSubsystem;
    m_robotDrive = driveSubsystem;
    
  }

  public Command getCommand() {

    return new InstantCommand(()->m_robotDrive.resetOdometry(path.getStartingDifferentialPose())).andThen(drive);
  }

  @Override
  public void periodic() {
    Pose2d tagPose = m_vision.pose3dRobotRelative().toPose2d();

    var poses = new ArrayList<Pose2d>();
    // poses.add(m_robotDrive.getPose());
    // poses.add(m_robotDrive.getPose().transformBy(new Transform2d(new Pose2d(0, 3, new Rotation2d()), tagPose)));

    poses.add(new Pose2d());
    poses.add(new Pose2d(0, .5, new Rotation2d()));

    //System.out.println(tagPose.toString() + "sdfsdfsdfsdfs");

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

    PathConstraints constraints = new PathConstraints(1, 1, 2 * Math.PI, 4 * Math.PI);

    var path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0, new Rotation2d()));



    path.preventFlipping = true;
    drive = AutoBuilder.followPath(path);
  }
}
