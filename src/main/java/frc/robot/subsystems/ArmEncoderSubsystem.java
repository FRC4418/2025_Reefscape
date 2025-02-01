// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmEncoderSubsystem extends SubsystemBase {
  public Encoder encoder = new Encoder(0, 3);
  /** Creates a new Encoder. */
  public ArmEncoderSubsystem() {
    
  }

  @Override
  public void periodic() {
    System.out.println(encoder.getDistance());
    // This method will be called once per scheduler run
  }
}
