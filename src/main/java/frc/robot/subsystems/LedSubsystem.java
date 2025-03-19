// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.UnitBuilder;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class LedSubsystem extends SubsystemBase {
  private AddressableLED m_led = new AddressableLED(0);
  // private AddressableLED m_led2 = new AddressableLED(1);
  private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(104);
  private AddressableLEDBufferView leftView = ledBuffer.createView(0, 51);
  private AddressableLEDBufferView rightView = ledBuffer.createView(52, 103).reversed();

  private LEDPattern red = LEDPattern.solid(new Color(0, 255, 0));
  // private LEDPattern green = LEDPattern.solid(new Color(0, 255, 0));
  private LEDPattern blue = LEDPattern.solid(new Color(0, 0, 255));

  private static final Distance kLedSpacing = Meters.of(1 / 120.0);

  private LEDPattern rainbow = LEDPattern.rainbow(255, 255);

  private LEDPattern enabledPattern = LEDPattern.rainbow(255, 255);

  /** Creates a new LedSubsystem. */
  public LedSubsystem() {
    rainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(.25), kLedSpacing);
    m_led.setLength(ledBuffer.getLength());
    // m_led2.setLength(ledBuffer.getLength());
    setPattern(red);
  }


  public void setPattern(LEDPattern pattern){

    pattern.applyTo(leftView);
    pattern.applyTo(rightView);
    m_led.setData(ledBuffer);
    m_led.start();
    // m_led2.setData(ledBuffer);
    // m_led2.start();
  }

  public void setEnabledPattern(LEDPattern pattern){
    enabledPattern = pattern;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(DriverStation.isTeleopEnabled()){
      setPattern(enabledPattern);
    }else if(DriverStation.isAutonomousEnabled()){
      setPattern(blue.breathe(Seconds.of(.5)));
    }else{
      setPattern(LEDPattern.solid(Color.kWhite));
    }
  }
}
