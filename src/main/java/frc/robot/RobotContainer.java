// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;

import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SetClimberPercentSpeed;
import frc.robot.commands.SetClimberPos;
import frc.robot.commands.SpinAlgaeSubsystem;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;
import frc.robot.subsystems.Manipulators.AlgaeSubsystem;
import frc.robot.subsystems.Manipulators.ClimberSubsystem;
import frc.robot.subsystems.Vision.ToAprilTag;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.utils.LimelightHelpers;

public class RobotContainer {
  
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // private final VisionSubsystem m_vision = new VisionSubsystem();

  private final ClimberSubsystem m_climber = new ClimberSubsystem();

  private final AlgaeSubsystem m_algeeSubsystem = new AlgaeSubsystem();

  // private final ToAprilTag toAprilTag = new ToAprilTag(m_vision, m_robotDrive);


  XboxController m_driverController = new XboxController(0);

  CommandXboxController m_CommandXboxControllerDriver = new CommandXboxController(0);

  public RobotContainer() {

    DataLogManager.start();

        m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

    configureBindings();

    // PathfindingCommand.warmupCommand().schedule();

    SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
      new Pose2d(10, 0, Rotation2d.fromDegrees(0)), 
      new PathConstraints(
        4.0, 4.0, 
        Units.degreesToRadians(360), Units.degreesToRadians(540)
      ), 
      0
    ));
  }



  private void configureBindings() {
    // m_CommandXboxControllerDriver.a().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
    m_CommandXboxControllerDriver.y().whileTrue(new SetClimberPercentSpeed(m_climber, -0.1));
    m_CommandXboxControllerDriver.x().whileTrue(new SetClimberPercentSpeed(m_climber, 0.1));

    m_CommandXboxControllerDriver.rightTrigger().whileTrue(new SpinAlgaeSubsystem(m_algeeSubsystem, 0, 1));

    m_CommandXboxControllerDriver.b().whileTrue(new SpinAlgaeSubsystem(m_algeeSubsystem, -0.3, -0.3));

    m_CommandXboxControllerDriver.a().whileTrue(new SpinAlgaeSubsystem(m_algeeSubsystem, 1, 0));

    m_climber.setDefaultCommand(new SetClimberPercentSpeed(m_climber, 0));

    m_algeeSubsystem.setDefaultCommand(new SpinAlgaeSubsystem(m_algeeSubsystem, 0, 0));

      // m_CommandXboxControllerDriver.b().whileTrue(Commands.runOnce(() -> {


    //   Pose2d tagPose = m_vision.pose3dRobotRelative().toPose2d();

    //   var poses = new ArrayList<Pose2d>();
    //   poses.add(m_robotDrive.getPose());
    //   Pose3d targetPose3d = m_vision.pose3dRobotRelative();

    //   // Pose2d targetPose = new Pose2d(-targetPose3d.getZ() + 1,-targetPose3d.getX(),m_robotDrive.getPose().getRotation());
    //   Pose2d targetPose = new Pose2d(1,0.2,new Rotation2d());

    //   Transform2d desiredTransform = new Transform2d(new Pose2d(), targetPose);
    //   Pose2d endPose = m_robotDrive.getPose().transformBy(desiredTransform);
    //   poses.add(endPose);




    //   System.out.println(tagPose.toString());

    //   List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

    //   PathConstraints constraints = new PathConstraints(4, 1, 2 * Math.PI, 4 * Math.PI);

    //   var path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0, new Rotation2d()));
    //   path.preventFlipping = true;



    //   // AutoBuilder.pathfindToPose(endPose, constraints,0d).schedule();;

    //   AutoBuilder.followPath(path).schedule();
    // }));
  }



  public Command getAutonomousCommand() {
    try{
        // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("New Path");

      // var path = PathPlannerPath.fromPathFile("Example Path");

      var traj = path.generateTrajectory(m_robotDrive.getRobotRelativeSpeeds(), m_robotDrive.getPose().getRotation(), m_robotDrive.config);
      



        // Create a path following command using AutoBuilder. This will also trigger event markers.
      return new InstantCommand(()->m_robotDrive.resetOdometry(path.getStartingDifferentialPose())).andThen(AutoBuilder.followPath(path));

    } catch (Exception e) {

      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }
}
