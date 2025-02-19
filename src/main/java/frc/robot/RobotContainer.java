// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.lang.instrument.Instrumentation;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.FileVersionException;

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
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldPositions;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ToggleCommand;
import frc.robot.commands.Algae.SetAlgaeIntakePercentSpeed;
import frc.robot.commands.Algae.SetAlgaePosition;
import frc.robot.commands.Algae.SetAlgaePositionMotorsPercentOutput;
import frc.robot.commands.Auto.DriveToPose;
import frc.robot.commands.Auto.DriveToTarget;
import frc.robot.commands.Climber.SetClimberPercentSpeed;
import frc.robot.commands.Climber.SetClimberPos;
import frc.robot.commands.Coral.SetCoralIntakePercentSpeed;
import frc.robot.commands.Coral.SetCoralPosition;
import frc.robot.commands.Coral.SetCoralPositionMotorsPercentOutput;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;
import frc.robot.subsystems.Manipulators.AlgaeSubsystem;
import frc.robot.subsystems.Manipulators.ClimberSubsystem;
import frc.robot.subsystems.Manipulators.CoralSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.RobotStateController;
import frc.utils.LimelightHelpers;

public class RobotContainer {
  
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final VisionSubsystem m_vision = new VisionSubsystem();

  private final ClimberSubsystem m_climber = new ClimberSubsystem();

  private final LedSubsystem m_led = new LedSubsystem();


  // private final AlgaeSubsystem m_algeeSubsystem = new AlgaeSubsystem();

  // private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();

  private final RobotStateController m_robotStateController = new RobotStateController(m_led, m_robotDrive);



  private XboxController m_driverController = new XboxController(0);

  private CommandXboxController m_CommandXboxControllerDriver = new CommandXboxController(0);
  
  private CommandXboxController m_CommandXboxControllerOther = new CommandXboxController(1);


  private SendableChooser<Command> chooser = new SendableChooser<Command>();

  public RobotContainer() {

    m_led.setPattern(LEDPattern.rainbow(255, 255));

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

    addAutoOptions();
  }

  private void configureBindings() {

    m_CommandXboxControllerOther.povDown().onTrue(new InstantCommand( () -> m_robotStateController.setTargetPose(FieldPositions.ABPose[1])));
    m_CommandXboxControllerOther.rightBumper().onTrue(new InstantCommand( () -> m_robotStateController.setTargetPose(FieldPositions.CDPose[1])));
    m_CommandXboxControllerOther.rightTrigger().onTrue(new InstantCommand( () -> m_robotStateController.setTargetPose(FieldPositions.EFPose[1])));
    m_CommandXboxControllerOther.povUp().onTrue(new InstantCommand( () -> m_robotStateController.setTargetPose(FieldPositions.GHPose[1])));
    m_CommandXboxControllerOther.leftTrigger().onTrue(new InstantCommand( () -> m_robotStateController.setTargetPose(FieldPositions.IJPose[1])));
    m_CommandXboxControllerOther.leftBumper().onTrue(new InstantCommand( () -> m_robotStateController.setTargetPose(FieldPositions.KLPose[1])));

    // Command outTake = new ToggleCommand(new SetCoralIntakePercentSpeed(m_coralSubsystem, 1), new SetAlgaeIntakePercentSpeed(m_algeeSubsystem, 0, 1), modeSupplier);

    m_CommandXboxControllerDriver.rightBumper().whileTrue(new RunCommand(() -> {
      m_robotStateController.setCoralMode(true); 
      // System.out.println(m_modeController.isInCoralMode());
    }));

    m_CommandXboxControllerDriver.leftBumper().whileTrue(new RunCommand(() -> {
      m_robotStateController.setCoralMode(false); 
      // System.out.println(m_modeController.isInCoralMode());
    }));

    

    SmartDashboard.putData("Reset Gyro", new InstantCommand( () -> m_robotDrive.zeroHeading() ));

    SmartDashboard.putData("Reset Pose Estimation", new InstantCommand( () -> m_robotDrive.resetPoseEstimation() ));

    m_CommandXboxControllerDriver.y().onTrue(new InstantCommand( () -> m_robotDrive.zeroTeleopHeading()));

    m_CommandXboxControllerDriver.a().toggleOnTrue(new DriveToTarget(m_robotDrive, m_robotStateController));




    // // m_CommandXboxControllerDriver.a().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));

    // m_CommandXboxControllerDriver.rightTrigger().whileTrue(new SetAlgaeIntakePercentSpeed(m_algeeSubsystem, 0, 1));

    // m_CommandXboxControllerDriver.b().whileTrue(new SetAlgaeIntakePercentSpeed(m_algeeSubsystem, -0.3, -0.3));

    // m_CommandXboxControllerDriver.a().whileTrue(new SetAlgaeIntakePercentSpeed(m_algeeSubsystem, 1, 0));

    m_climber.setDefaultCommand(new SetClimberPercentSpeed(m_climber, 0));

    // m_algeeSubsystem.setDefaultCommand(new SetAlgaeIntakePercentSpeed(m_algeeSubsystem, 0, 0).alongWith(new SetAlgaePositionMotorsPercentOutput(m_algeeSubsystem, 0, 0)));

    // m_coralSubsystem.setDefaultCommand(new SetCoralIntakePercentSpeed(m_coralSubsystem, 0).alongWith(new SetCoralPositionMotorsPercentOutput(m_coralSubsystem, 0, 0)));
  }

  public void addAutoOptions(){
    chooser.addOption("None", new InstantCommand());

    chooser.setDefaultOption("test", getTestCommand());

    SmartDashboard.putData("Auto Selector", chooser);
  }

  public PathPlannerPath getPath(String name){
    try {
      var path =  PathPlannerPath.fromPathFile(name);


      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        if (alliance.get() == DriverStation.Alliance.Red) return path.flipPath();
      }

      return path;

    } catch (FileVersionException | IOException | ParseException e) {
      return null;
    }
  }

  public Command getTestCommand(){

    PathPlannerPath path = getPath("test");

    Command drivePath = AutoBuilder.followPath(path);

    Command resetPose = new InstantCommand(() -> m_robotDrive.resetOdometry(path.getStartingDifferentialPose()));

    return resetPose.andThen(drivePath);
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }
}
