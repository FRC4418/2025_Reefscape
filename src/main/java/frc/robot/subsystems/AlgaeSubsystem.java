// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class AlgeeSubsystem extends SubsystemBase {
  /** Creates a new AlgeeSubsystem. */
  
  private final SparkMax m_intakeMotor1 = new SparkMax(20, MotorType.kBrushless);
  private final SparkMax m_intakeMotor2 = new SparkMax(21, MotorType.kBrushless);
  private final SparkMax m_shootMotor1 = new SparkMax(23, MotorType.kBrushless);
  private final SparkMax m_shootMotor2 = new SparkMax(24, MotorType.kBrushless);
  public AlgeeSubsystem() {
    SparkBaseConfig invertedConfig = new SparkMaxConfig().inverted(true);
    SparkBaseConfig defautConfig = new SparkMaxConfig().inverted(false);
    m_intakeMotor1.configure(defautConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_intakeMotor2.configure(defautConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_shootMotor1.configure(defautConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_shootMotor2.configure(defautConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void spinShooters(double speed){
    m_shootMotor1.set(speed);
    m_shootMotor2.set(speed);
  }

  public void spinIntake(double speed){
    m_intakeMotor1.set(speed);
    m_intakeMotor2.set(speed);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
