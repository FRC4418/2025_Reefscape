// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldPositions;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;
import frc.robot.subsystems.Manipulators.CoralSubsystem;

public class RobotStateController extends SubsystemBase {
  private boolean coralMode;
  private LedSubsystem m_ledSubsystem;
  private DriveSubsystem m_robotDrive;
  private CoralSubsystem m_coralSubsystem;

  private Pose2d targetPose2d = FieldPositions.GHPose[1];

  private Pose2d intakePose = FieldPositions.leftIntakeBlue;

  private Transform2d scoreMovementTransform = new Transform2d();

  private double elevatorScorePos = 0;

  private double wristScorePos = 0.2;

  private boolean isOnRed = false;

  private LEDPattern redGrad = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, new Color(0,255,0), new Color(100,255,0));

  private LEDPattern blueGrad = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, new Color(255,0,200), new Color(0,100,255));

  private LEDPattern teleopDefault = blueGrad;

  Distance ledSpacing = Meters.of(1 / 120.0);


  LEDPattern base;

  /** Creates a new ModeController. */
  public RobotStateController(LedSubsystem ledSubsystem, CoralSubsystem coralSubsystem, DriveSubsystem driveSubsystem) {
    m_ledSubsystem = ledSubsystem;
    m_robotDrive = driveSubsystem;
    m_coralSubsystem = coralSubsystem;
    
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      isOnRed =  alliance.get() == DriverStation.Alliance.Red;
    }
    if(!isOnRed){
      driveSubsystem.absoluteGyroOffset = 180;
    }

    
    base = (isOnRed ? redGrad : blueGrad);

    base = base.scrollAtRelativeSpeed(Hertz.of(.5));
    
    m_ledSubsystem.setEnabledPattern(base);
  }

  public void setBaseLEDPattern(LEDPattern ledPattern){
    m_ledSubsystem.setEnabledPattern(ledPattern);
  }

  // public void setLedTeleopPattern(le)

  public void setCoralMode(boolean mode){
    coralMode = mode;
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




    // if(m_coralSubsystem.hasCoral()){
    //   setBaseLEDPattern(base.blink(Units.Second.of(.5)));
    // }else{
    //   setBaseLEDPattern(base);
    // }

    Logger.recordOutput("Elevator Target Pose", getElevatorScorePos());

    Logger.recordOutput("Intake Target Pose", intakePose);
    // System.out.println(coralMode);
    // This method will be called once per scheduler run
    Logger.recordOutput("Target Pose", targetPose2d);

    Logger.recordOutput("Score Pose", targetPose2d.transformBy(scoreMovementTransform));
  }
}
