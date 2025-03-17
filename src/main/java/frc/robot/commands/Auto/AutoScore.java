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
import frc.robot.Constants.FieldPositions;
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
  private Command drivePath1;
  private Command drivePath2;
  private Command mainCommand;

  private boolean autoEnding = false;

  private boolean use2Paths = false;

  private boolean doneWithPath1 = false;
  private boolean doneWithPath2 = false;
  

  /** Creates a new AutoScore. */
  public AutoScore(DriveSubsystem driveSubsystem, CoralSubsystem coralSubsystem, RobotStateController robotStateController) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(robotStateController, driveSubsystem, coralSubsystem);
    this.m_robotDrive = driveSubsystem;
    this.m_robotStateController = robotStateController;
    this.m_coralSubsystem = coralSubsystem;
  }
  public AutoScore(DriveSubsystem driveSubsystem, CoralSubsystem coralSubsystem, RobotStateController robotStateController, boolean use2Paths) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(robotStateController, driveSubsystem, coralSubsystem);
    this.m_robotDrive = driveSubsystem;
    this.m_robotStateController = robotStateController;
    this.m_coralSubsystem = coralSubsystem;
    this.use2Paths = use2Paths;
  }
  public AutoScore(DriveSubsystem driveSubsystem, CoralSubsystem coralSubsystem, RobotStateController robotStateController, boolean use2Paths, boolean autoEnding) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(robotStateController, driveSubsystem, coralSubsystem);
    this.m_robotDrive = driveSubsystem;
    this.m_robotStateController = robotStateController;
    this.use2Paths = use2Paths;
    this.m_coralSubsystem = coralSubsystem;
    this.autoEnding = autoEnding;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    drivePath1 = AutoBuilder.followPath(getTargetPath());

    drivePath2 = AutoBuilder.followPath(getScorePath());
    

    mainCommand = AutoBuilder.followPath(getPath(m_robotDrive.getPose(), m_robotStateController.getTargetPose().transformBy(m_robotStateController.getScoreTransform()).transformBy(FieldPositions.scoreOffset)));
    
    mainCommand.initialize();

    if(autoEnding) mainCommand.schedule();
    // drivePath2.initialize();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!autoEnding) mainCommand.execute();
    // if(doneWithPath1){
    //   part1();
    // }else{
    //   part2();
    // }
  }

  // private void part1(){

  //   drivePath1.execute();
    
  //   if(drivePath1.isFinished()){ 
  //     doneWithPath1 = true;
  //   }
  // }

  // private void part2(){
  //   drivePath2.execute();
  //   // m_coralSubsystem.setManipulatorPos(m_robotStateController.getElevatorScorePos(), m_robotStateController.getWristScorePos());
  // }

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
    poses.add(m_robotStateController.getTargetPose().transformBy(m_robotStateController.getScoreTransform()).transformBy(FieldPositions.scoreOffset));

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

    PathConstraints constraints = new PathConstraints(1.5, 1, 2 * Math.PI, 4 * Math.PI);

    return new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0, m_robotStateController.getTargetPose().getRotation()));
  }

  public PathPlannerPath getPath(Pose2d initPose, Pose2d finalPose){
    
    var poses = new ArrayList<Pose2d>();

    poses.add(initPose);
    poses.add(finalPose);

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

    PathConstraints constraints = new PathConstraints(1, .5, 2 * Math.PI, 4 * Math.PI);

    return new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0, m_robotStateController.getTargetPose().getRotation()));
  }
}
