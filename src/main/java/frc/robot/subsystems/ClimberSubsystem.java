// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class ClimberSubsystem extends SubsystemBase {

  private final SparkMax m_climberSparkMax = new SparkMax(MotorIDs.climberMotorID, MotorType.kBrushless);
  private final RelativeEncoder m_encoder;
  private final SparkClosedLoopController m_controller;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_encoder = m_climberSparkMax.getEncoder();
    m_controller = m_climberSparkMax.getClosedLoopController();


    m_climberSparkMax.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setPosition(double pos){
    m_controller.setReference(pos, ControlType.kPosition);
  }

  public void setPercentSpeed(double percent){
    m_climberSparkMax.set(percent);
  }



  @Override
  public void periodic() {
    // System.out.println(m_encoder.getPosition());
    // This method will be called once per scheduler run
  }
}
