// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ModeController extends SubsystemBase {
  private boolean coralMode;
  /** Creates a new ModeController. */
  public ModeController() {}

  public void setCoralMode(boolean mode){
    coralMode = mode;
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
