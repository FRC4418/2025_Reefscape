// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Manipulators;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ManipulatorGearRatios;
import frc.robot.Constants.MotorIDs;

public class AlgaeSubsystem extends SubsystemBase {
  /** Creates a new AlgeeSubsystem. */
  
  private final SparkMax m_intakeMotor1 = new SparkMax(MotorIDs.leftAlgaeIntakeMotorID, MotorType.kBrushless);
  private final SparkMax m_intakeMotor2 = new SparkMax(MotorIDs.rightAlgaeIntakeMotorID, MotorType.kBrushless);

  private final SparkMax m_wristMotor = new SparkMax(MotorIDs.algaeWristMotorID, MotorType.kBrushless);
  private final AbsoluteEncoder m_wristAbsoluteEncoder = m_wristMotor.getAbsoluteEncoder();

  private final SparkMax m_leftElevatorMotor = new SparkMax(MotorIDs.leftAlgaeElevatorMotorID, MotorType.kBrushless);
  private final SparkMax m_rightElevatorMotor = new SparkMax(MotorIDs.rightAlgaeElevatorMotorID, MotorType.kBrushless);
  private final RelativeEncoder m_elevatorEncoder = m_leftElevatorMotor.getEncoder();

  public AlgaeSubsystem() {

  }

  public void spinIntake(double speed){
    m_intakeMotor1.set(speed);
    m_intakeMotor2.set(speed);
  }

  public double getWristPos(){
    return m_wristAbsoluteEncoder.getPosition();
  }

  public void setWristPercentOutput(double speed){
    m_wristMotor.set(speed);
  }

  public double getElevatorPos(){
    return m_elevatorEncoder.getPosition();
  }

  public void setElevatorPercentOutput(double speed){
    m_leftElevatorMotor.set(speed);
    m_rightElevatorMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
