// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.text.FieldPosition;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldPositions;
import frc.robot.Constants.ManipulatorPositions;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Auto.AutoIntake;
import frc.robot.commands.Auto.AutoScore;
import frc.robot.commands.Climber.SetClimberPercentSpeed;
import frc.robot.commands.Coral.CoralDefault;
import frc.robot.commands.Coral.IntakeUntillGood;
import frc.robot.commands.Coral.SetCoralIntakePercentSpeed;
import frc.robot.commands.Coral.SetCoralPosition;
import frc.robot.commands.Coral.SetCoralPositionMotorsPercentOutput;
import frc.robot.commands.Coral.SetCoralPositionToTarget;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.RobotStateController;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;
import frc.robot.subsystems.Manipulators.ClimberSubsystem;
import frc.robot.subsystems.Manipulators.CoralSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;

public class RobotContainer {
  
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private final VisionSubsystem m_vision = new VisionSubsystem();

  private final ClimberSubsystem m_climber = new ClimberSubsystem();

  private final LedSubsystem m_led = new LedSubsystem();

  private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();

  private final RobotStateController m_robotStateController = new RobotStateController(m_led, m_coralSubsystem, m_robotDrive);



  private XboxController m_driverController = new XboxController(0);

  private CommandXboxController m_CommandXboxControllerDriver = new CommandXboxController(0);
  
  private CommandXboxController m_CommandXboxControllerOther = new CommandXboxController(1);

  private CommandGenericHID m_buttonBoard = new CommandGenericHID(5);



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


    m_buttonBoard.button(1).onTrue(new InstantCommand(() -> m_robotStateController.setTargetAndScorePos(FieldPositions.ABPose, false)));
    m_buttonBoard.button(2).onTrue(new InstantCommand(() -> m_robotStateController.setTargetAndScorePos(FieldPositions.KLPose, true)));
    m_buttonBoard.button(3).onTrue(new InstantCommand(() -> m_robotStateController.setTargetAndScorePos(FieldPositions.KLPose, false)));
    m_buttonBoard.button(4).onTrue(new InstantCommand(() -> m_robotStateController.setTargetAndScorePos(FieldPositions.IJPose, true)));
    m_buttonBoard.button(5).onTrue(new InstantCommand(() -> m_robotStateController.setTargetAndScorePos(FieldPositions.IJPose, false)));
    m_buttonBoard.button(6).onTrue(new InstantCommand(() -> m_robotStateController.setTargetAndScorePos(FieldPositions.GHPose, true)));
    m_buttonBoard.button(7).onTrue(new InstantCommand(() -> m_robotStateController.setTargetAndScorePos(FieldPositions.GHPose, false)));
    m_buttonBoard.button(8).onTrue(new InstantCommand(() -> m_robotStateController.setTargetAndScorePos(FieldPositions.EFPose, true)));
    m_buttonBoard.button(9).onTrue(new InstantCommand(() -> m_robotStateController.setTargetAndScorePos(FieldPositions.EFPose, false)));
    m_buttonBoard.button(10).onTrue(new InstantCommand(() -> m_robotStateController.setTargetAndScorePos(FieldPositions.CDPose, true)));
    m_buttonBoard.button(11).onTrue(new InstantCommand(() -> m_robotStateController.setTargetAndScorePos(FieldPositions.CDPose, false)));
    m_buttonBoard.button(12).onTrue(new InstantCommand(() -> m_robotStateController.setTargetAndScorePos(FieldPositions.ABPose, true)));

    m_buttonBoard.axisGreaterThan(0, 0.6).onTrue(new InstantCommand(() ->m_robotStateController.setScoreManipulatorPos(ManipulatorPositions.kCoralElevatorPosL2, ManipulatorPositions.kCoralWristPosL2)));
    m_buttonBoard.axisGreaterThan(1, 0.6).onTrue(new InstantCommand(() ->m_robotStateController.setScoreManipulatorPos(ManipulatorPositions.kCoralElevatorPosL3, ManipulatorPositions.kCoralWristPosL3)));
    m_buttonBoard.axisLessThan(1, -0.6).onTrue(new InstantCommand(() ->m_robotStateController.setScoreManipulatorPos(ManipulatorPositions.kCoralElevatorPosL4, ManipulatorPositions.kCoralWristPosL4)));

    m_CommandXboxControllerOther.povDown().onTrue(new InstantCommand( () -> m_robotStateController.setTargetPose(FieldPositions.ABPose)));
    m_CommandXboxControllerOther.rightBumper().onTrue(new InstantCommand( () -> m_robotStateController.setTargetPose(FieldPositions.CDPose)));
    m_CommandXboxControllerOther.rightTrigger().onTrue(new InstantCommand( () -> m_robotStateController.setTargetPose(FieldPositions.EFPose)));
    m_CommandXboxControllerOther.povUp().onTrue(new InstantCommand( () -> m_robotStateController.setTargetPose(FieldPositions.GHPose)));
    m_CommandXboxControllerOther.leftTrigger().onTrue(new InstantCommand( () -> m_robotStateController.setTargetPose(FieldPositions.IJPose)));
    m_CommandXboxControllerOther.leftBumper().onTrue(new InstantCommand( () -> m_robotStateController.setTargetPose(FieldPositions.KLPose)));

    m_CommandXboxControllerOther.povLeft().onTrue(new InstantCommand( () -> m_robotStateController.setScoreTransform(true)));
    m_CommandXboxControllerOther.povRight().onTrue(new InstantCommand( () -> m_robotStateController.setScoreTransform(false)));
    // Command outTake = new ToggleCommand(new SetCoralIntakePercentSpeed(m_coralSubsystem, 1), new SetAlgaeIntakePercentSpeed(m_algeeSubsystem, 0, 1), modeSupplier);

    m_CommandXboxControllerOther.rightBumper().onTrue(new InstantCommand( () -> m_robotStateController.setScoreManipulatorPos(ManipulatorPositions.kCoralElevatorPosL2, ManipulatorPositions.kCoralWristPosL2) ));
    m_CommandXboxControllerOther.leftTrigger().onTrue(new InstantCommand( () -> m_robotStateController.setScoreManipulatorPos(ManipulatorPositions.kCoralElevatorPosL3, ManipulatorPositions.kCoralWristPosL3) ));
    m_CommandXboxControllerOther.rightTrigger().onTrue(new InstantCommand( () -> m_robotStateController.setScoreManipulatorPos(ManipulatorPositions.kCoralElevatorPosL4, ManipulatorPositions.kCoralWristPosL4) ));


    m_CommandXboxControllerOther.a().whileTrue(new SetClimberPercentSpeed(m_climber, -.4));
    m_CommandXboxControllerOther.y().whileTrue(new SetClimberPercentSpeed(m_climber, .4));
    

    

    SmartDashboard.putData("Reset Gyro", new InstantCommand( () -> m_robotDrive.zeroHeading() ));

    SmartDashboard.putData("Reset Pose Estimation", new InstantCommand( () -> m_robotDrive.resetPoseEstimation() ));

    SmartDashboard.putData("Reset Pose to Targer", new InstantCommand( () -> m_robotDrive.resetOdometry(m_robotStateController.getTargetPose()) ));

    m_CommandXboxControllerDriver.y().onTrue(new InstantCommand( () -> m_robotDrive.zeroTeleopHeading()));

    m_CommandXboxControllerDriver.a().toggleOnTrue(new AutoScore(m_robotDrive, m_coralSubsystem, m_robotStateController, false).alongWith(new SetCoralPositionToTarget(m_coralSubsystem, m_robotStateController)));

    m_CommandXboxControllerDriver.b().whileTrue(new SetCoralIntakePercentSpeed(m_coralSubsystem, 1));

    m_CommandXboxControllerDriver.x().whileTrue(new SetCoralIntakePercentSpeed(m_coralSubsystem, -1));

    // m_CommandXboxControllerDriver.rightBumper().toggleOnTrue(new AutoIntake(m_robotDrive, m_robotStateController, m_coralSubsystem).
    //   raceWith(new IntakeUntillGood(m_coralSubsystem, 1))  );

    m_CommandXboxControllerDriver.povUp().toggleOnTrue(new SetCoralPosition(m_coralSubsystem, ManipulatorPositions.kCoralElevatorPosL4, ManipulatorPositions.kCoralWristPosL4));
    m_CommandXboxControllerDriver.povRight().toggleOnTrue(new SetCoralPosition(m_coralSubsystem, ManipulatorPositions.kCoralElevatorPosL3, ManipulatorPositions.kCoralWristPosL3));
    m_CommandXboxControllerDriver.povLeft().toggleOnTrue(new SetCoralPosition(m_coralSubsystem, ManipulatorPositions.kCoralElevatorPosL2, ManipulatorPositions.kCoralWristPosL2));
    m_CommandXboxControllerDriver.povDown().whileTrue(new SetCoralPosition(m_coralSubsystem, 0.1, 0.135));

    // m_CommandXboxControllerDriver.povUp().whileTrue(new SetCoralPositionMotorsPercentOutput(m_coralSubsystem, 0.2, 0));

    // m_CommandXboxControllerDriver.rightBumper().toggleOnTrue(new AutoIntake(m_robotDrive, m_robotStateController, m_coralSubsystem));

    m_CommandXboxControllerDriver.rightBumper().toggleOnTrue(new IntakeUntillGood(m_coralSubsystem, -.5));


    m_CommandXboxControllerDriver.rightTrigger().whileTrue(new SetClimberPercentSpeed(m_climber, 0.9));
    m_CommandXboxControllerDriver.leftTrigger().whileTrue(new SetClimberPercentSpeed(m_climber, -0.9));

    m_climber.setDefaultCommand(new SetClimberPercentSpeed(m_climber, 0));

    m_coralSubsystem.setDefaultCommand(new CoralDefault(m_coralSubsystem));
    // m_coralSubsystem.setDefaultCommand(new SetCoralIntakePercentSpeed(m_coralSubsystem, 0).alongWith(new SetCoralPositionMotorsPercentOutput(m_coralSubsystem, 0, 0)));
  }

  public void addAutoOptions(){
    chooser.setDefaultOption("None", new InstantCommand());

    chooser.addOption("center1p", center1p());
    chooser.addOption("left1p", left1p());
    chooser.addOption("right1p", right1p());

    chooser.addOption("Forward",getGoForward());
    chooser.addOption("Left Forward",getGoForwardLeft());
    chooser.addOption("Right Forward",getGoForwardRight());

    chooser.addOption("test", getTestCommand());

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

  public Command getDrivePath(String name){
    return new InstantCommand();
  }

  public Command getGoForward(){
    var path = getPath("Forward");

    Command resetPose = new InstantCommand(() -> m_robotDrive.resetOdometry(path.getStartingDifferentialPose()));

    // return resetPose.andThen(AutoBuilder.followPath(path));
    return resetPose.andThen(m_robotDrive.followPathCommand("Forward"));
  }
  public Command getGoForwardLeft(){
    var path = getPath("Left Forward");

    Command resetPose = new InstantCommand(() -> m_robotDrive.resetOdometry(path.getStartingDifferentialPose()));

    return resetPose.andThen(AutoBuilder.followPath(path));
  }
  public Command getGoForwardRight(){
    var path = getPath("Right Forward");

    Command resetPose = new InstantCommand(() -> m_robotDrive.resetOdometry(path.getStartingDifferentialPose()));

    return resetPose.andThen(AutoBuilder.followPath(path));
  }

  public Command center1p(){
    var path = getPath("Forward");

    Command resetPose = new InstantCommand(() -> m_robotDrive.resetOdometry(path.getStartingDifferentialPose()));

    Command driveForward = AutoBuilder.followPath(path);

    
    var path2 = getPath("Go Back");


    Command driveBack = AutoBuilder.followPath(path2);
    
    // Command setupScorePos = new InstantCommand( () -> m_robotStateController.setScoreManipulatorPos(ManipulatorPositions.kCoralElevatorPosL4, ManipulatorPositions.kCoralWristPosL4) )
    // .alongWith( new InstantCommand( () -> m_robotStateController.setTargetAndScorePos(FieldPositions.GHPose, false) ))
    // .andThen(new WaitCommand(0.5));
    // Command score = new AutoScore(m_robotDrive, m_coralSubsystem, m_robotStateController, false, true).
    // alongWith(new SetCoralPosition(m_coralSubsystem, ManipulatorPositions.kCoralElevatorPosL3, ManipulatorPositions.kCoralWristPosL3));

    Command outTake = new SetCoralIntakePercentSpeed(m_coralSubsystem, 1).alongWith(new SetCoralPosition(m_coralSubsystem, 0, 0.2)).raceWith(new WaitCommand(3));

    // return getGoForward().andThen(setupScorePos.andThen(score).andThen(outTake));

    Command setTarget = new InstantCommand(() -> m_robotStateController.setTargetAndScorePos(FieldPositions.GHPose, false));

    Command align = new AutoScore(m_robotDrive, m_coralSubsystem, m_robotStateController, false, true).raceWith(new SetCoralPosition(m_coralSubsystem, ManipulatorPositions.kCoralElevatorPosL4, ManipulatorPositions.kCoralWristPosL4));



    return new SequentialCommandGroup(resetPose, setTarget, align, outTake);
  }


  public Command right1p(){
    
    var path = getPath("Right 1");

    Command resetPose = new InstantCommand(() -> m_robotDrive.resetOdometry(path.getStartingDifferentialPose()));

    Command driveForward = AutoBuilder.followPath(path);

    
    var path2 = getPath("Go Back");


    Command driveBack = AutoBuilder.followPath(path2);
    
    // Command setupScorePos = new InstantCommand( () -> m_robotStateController.setScoreManipulatorPos(ManipulatorPositions.kCoralElevatorPosL4, ManipulatorPositions.kCoralWristPosL4) )
    // .alongWith( new InstantCommand( () -> m_robotStateController.setTargetAndScorePos(FieldPositions.GHPose, false) ))
    // .andThen(new WaitCommand(0.5));
    // Command score = new AutoScore(m_robotDrive, m_coralSubsystem, m_robotStateController, false, true).
    // alongWith(new SetCoralPosition(m_coralSubsystem, ManipulatorPositions.kCoralElevatorPosL3, ManipulatorPositions.kCoralWristPosL3));

    Command outTake = new SetCoralIntakePercentSpeed(m_coralSubsystem, 1).alongWith(new SetCoralPosition(m_coralSubsystem, 0, 0.2)).raceWith(new WaitCommand(3));

    // return getGoForward().andThen(setupScorePos.andThen(score).andThen(outTake));

    Command setTarget = new InstantCommand(() -> m_robotStateController.setTargetAndScorePos(FieldPositions.EFPose, false));

    Command align = new AutoScore(m_robotDrive, m_coralSubsystem, m_robotStateController, false, true).raceWith(new SetCoralPosition(m_coralSubsystem, ManipulatorPositions.kCoralElevatorPosL4, ManipulatorPositions.kCoralWristPosL4));



    return new SequentialCommandGroup(resetPose, setTarget, align, outTake);
  }

  public Command left1p(){

    var path = getPath("Left 1");

    Command resetPose = new InstantCommand(() -> m_robotDrive.resetOdometry(path.getStartingDifferentialPose()));

    Command driveForward = AutoBuilder.followPath(path);

    
    var path2 = getPath("Go Back");


    Command driveBack = AutoBuilder.followPath(path2);
    
    // Command setupScorePos = new InstantCommand( () -> m_robotStateController.setScoreManipulatorPos(ManipulatorPositions.kCoralElevatorPosL4, ManipulatorPositions.kCoralWristPosL4) )
    // .alongWith( new InstantCommand( () -> m_robotStateController.setTargetAndScorePos(FieldPositions.GHPose, false) ))
    // .andThen(new WaitCommand(0.5));
    // Command score = new AutoScore(m_robotDrive, m_coralSubsystem, m_robotStateController, false, true).
    // alongWith(new SetCoralPosition(m_coralSubsystem, ManipulatorPositions.kCoralElevatorPosL3, ManipulatorPositions.kCoralWristPosL3));

    Command outTake = new SetCoralIntakePercentSpeed(m_coralSubsystem, 1).alongWith(new SetCoralPosition(m_coralSubsystem, 0, 0.2)).raceWith(new WaitCommand(3));

    // return getGoForward().andThen(setupScorePos.andThen(score).andThen(outTake));

    Command setTarget = new InstantCommand(() -> m_robotStateController.setTargetAndScorePos(FieldPositions.IJPose, false));

    Command align = new AutoScore(m_robotDrive, m_coralSubsystem, m_robotStateController, false, true).raceWith(new SetCoralPosition(m_coralSubsystem, ManipulatorPositions.kCoralElevatorPosL4, ManipulatorPositions.kCoralWristPosL4));



    return new SequentialCommandGroup(resetPose, setTarget, align, outTake);
  }
  
 
  public Command getTestCommand(){

    PathPlannerPath path = getPath("test");

    Command drivePath = AutoBuilder.followPath(path);

    Command resetPose = new InstantCommand(() -> m_robotDrive.resetOdometry(path.getStartingDifferentialPose()));

    return resetPose.andThen(drivePath);
  }
  
  public Command getAutonomousCommand() {
    return chooser.getSelected();
    // return center1p();
    // return getGoForward().andThen(center1p());
  }
}
