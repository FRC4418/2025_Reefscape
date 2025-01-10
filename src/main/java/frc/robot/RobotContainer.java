// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;

public class RobotContainer {
  
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();


  XboxController m_driverController = new XboxController(0);

  CommandXboxController m_CommandXboxControllerDriver = new CommandXboxController(0);

  public RobotContainer() {

        m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

    configureBindings();
  }

  private void configureBindings() {
    m_CommandXboxControllerDriver.a().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
  }

  public Command getAutonomousCommand() {
    try{
        // Load the path you want to follow using its name in the GUI
      // PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("New Path");

      var path = PathPlannerPath.fromPathFile("Example Path");

      var traj = path.generateTrajectory(m_robotDrive.getRobotRelativeSpeeds(), m_robotDrive.getPose().getRotation(), m_robotDrive.config);
      



        // Create a path following command using AutoBuilder. This will also trigger event markers.
      return new InstantCommand(()->m_robotDrive.resetOdometry(path.getStartingDifferentialPose())).andThen(AutoBuilder.followPath(path));

    } catch (Exception e) {

      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      return Commands.none();
    }
  }
}
