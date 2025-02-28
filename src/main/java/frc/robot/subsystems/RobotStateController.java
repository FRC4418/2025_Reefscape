// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import java.text.FieldPosition;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldPositions;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;

public class RobotStateController extends SubsystemBase {
  private boolean coralMode;
  private LedSubsystem m_ledSubsystem;
  private DriveSubsystem m_robotDrive;

  private Pose2d targetPose2d = FieldPositions.GHPose[1];

  private Pose2d intakePose = FieldPositions.leftIntakeBlue;

  private Transform2d scoreMovementTransform = new Transform2d();

  private double elevatorScorePos = 0;

  private double wristScorePos = 0.2;

  private boolean isOnRed = false;

  
  

  private LEDPattern coralLEDPattern = LEDPattern.solid(new Color(255, 255, 0));
  private LEDPattern algaeLEDPattern = LEDPattern.solid(new Color(255, 0, 100));

  /** Creates a new ModeController. */
  public RobotStateController(LedSubsystem ledSubsystem, DriveSubsystem driveSubsystem) {
    m_ledSubsystem = ledSubsystem;
    m_robotDrive = driveSubsystem;

    
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      isOnRed =  alliance.get() == DriverStation.Alliance.Red;
    }
    if(isOnRed){
      driveSubsystem.absoluteGyroOffset = 180;
    }
    
  }

  public void setCoralMode(boolean mode){
    coralMode = mode;
    if(mode){
      m_ledSubsystem.setEnabledPattern(coralLEDPattern);
      m_robotDrive.setDriveYawOffset(0);
      
    }else{
      m_ledSubsystem.setEnabledPattern(algaeLEDPattern);
      m_robotDrive.setDriveYawOffset(90);

    }
  }
  public boolean isRed(){
    return isOnRed;
  }

  public void setScoreTransform(boolean left){
    scoreMovementTransform = (left ? Constants.FieldPositions.leftScoreTransform :  Constants.FieldPositions.rightScoreTransform);
  }

  public void setScoreManipulatorPos(double elevator, double wrist){
    elevatorScorePos = elevator;
    wristScorePos = wrist;
  }

  public double getWristScorePos(){
    return wristScorePos;
  }

  public double getElevatorScorePos(){
    return elevatorScorePos;
  }

  public Transform2d getScoreTransform(){
    return scoreMovementTransform;
  }

  public void setTargetPose(Pose2d[] pose){
    targetPose2d = isOnRed ? pose[1] : pose[0];
  }

  public void setTargetAndScorePos(Pose2d[] poes, boolean left){
    setTargetPose(poes);
    setScoreTransform(left);
  }

  public Pose2d getTargetPose(){
    return targetPose2d;
  }

  public boolean isInCoralMode(){
    return coralMode;
  }

  public Pose2d getIntakPos(){
    return intakePose;
  }

  @Override
  public void periodic() {
    if(m_robotDrive.getPose().getY() > 4){
      intakePose = isOnRed ? FieldPositions.leftIntakeRed : FieldPositions.rightIntakeBlue;
    }else{
      intakePose = isOnRed ? FieldPositions.rightIntakeRed : FieldPositions.leftIntakeBlue;
    }
    Logger.recordOutput("Elevator Target Pose", getElevatorScorePos());

    Logger.recordOutput("Intake Target Pose", intakePose);
    // System.out.println(coralMode);
    // This method will be called once per scheduler run
    Logger.recordOutput("Target Pose", targetPose2d);

    Logger.recordOutput("Score Pose", targetPose2d.transformBy(scoreMovementTransform));
  }
}
