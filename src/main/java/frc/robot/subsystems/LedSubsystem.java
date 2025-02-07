// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class LedSubsystem extends SubsystemBase {
  private AddressableLED m_led = new AddressableLED(3);
  private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(300);

  private LEDPattern red = LEDPattern.solid(new Color(255, 0, 0));
  private LEDPattern green = LEDPattern.solid(new Color(0, 255, 0));
  private LEDPattern blue = LEDPattern.solid(new Color(255, 0, 255));

  /** Creates a new LedSubsystem. */
  public LedSubsystem() {
    setPattern(red);
  }


  public void setPattern(LEDPattern pattern){
    pattern.applyTo(ledBuffer);
    m_led.setData(ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(DriverStation.isTeleopEnabled()){
      setPattern(green);
    }else if(DriverStation.isAutonomousEnabled()){
      setPattern(blue);
    }else{
      setPattern(red);
    }
  }
}
