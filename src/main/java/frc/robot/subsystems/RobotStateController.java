// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotStateController extends SubsystemBase {
  private boolean coralMode;
  private LedSubsystem m_ledSubsystem;

  private LEDPattern coralLEDPattern = LEDPattern.solid(new Color(200, 200, 200));
  private LEDPattern algaeLEDPattern = LEDPattern.solid(new Color(255, 0, 200));

  /** Creates a new ModeController. */
  public RobotStateController(LedSubsystem ledSubsystem) {
    m_ledSubsystem = ledSubsystem;
  }

  public void setCoralMode(boolean mode){
    coralMode = mode;
    if(mode){
      m_ledSubsystem.setEnabledPattern(coralLEDPattern);
    }else{
      m_ledSubsystem.setEnabledPattern(algaeLEDPattern);
    }
  }

  public boolean isInCoralMode(){
    return coralMode;
  }

  @Override
  public void periodic() {
    // System.out.println(coralMode);
    // This method will be called once per scheduler run
  }
}
