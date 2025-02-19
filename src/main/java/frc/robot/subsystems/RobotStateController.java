// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldPositions;
import frc.robot.subsystems.Drivetrain.DriveSubsystem;

public class RobotStateController extends SubsystemBase {
  private boolean coralMode;
  private LedSubsystem m_ledSubsystem;
  private DriveSubsystem m_robotDrive;

  private Pose2d targetPose2d = FieldPositions.GHPose[1];

  
  

  private LEDPattern coralLEDPattern = LEDPattern.solid(new Color(255, 255, 0));
  private LEDPattern algaeLEDPattern = LEDPattern.solid(new Color(255, 0, 100));

  /** Creates a new ModeController. */
  public RobotStateController(LedSubsystem ledSubsystem, DriveSubsystem driveSubsystem) {
    m_ledSubsystem = ledSubsystem;
    m_robotDrive = driveSubsystem;

    boolean isOnRed = false;
    
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      isOnRed =  alliance.get() == DriverStation.Alliance.Red;
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

  public void setTargetPose(Pose2d pose){
    targetPose2d = pose;
  }


  public Pose2d getTargetPose(){
    return targetPose2d;
  }

  public boolean isInCoralMode(){
    return coralMode;
  }

  @Override
  public void periodic() {
    // System.out.println(coralMode);
    // This method will be called once per scheduler run
    Logger.recordOutput("Target Pose", targetPose2d);
  }
}
