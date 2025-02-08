// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Manipulators;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorGearRatios;
import frc.robot.Constants.MotorIDs;

public class CoralSubsystem extends SubsystemBase {
  private final TalonFX m_wristMotor = new TalonFX(MotorIDs.coralWristMotorID);

  private final SparkMax m_coralMotor = new SparkMax(MotorIDs.coralMotorID, MotorType.kBrushless);
  private final AbsoluteEncoder m_wristAbsoluteEncoder = m_coralMotor.getAbsoluteEncoder();

  private final SparkMax m_leftElevatorMotor = new SparkMax(MotorIDs.leftCoralElevatorMotorID, MotorType.kBrushless);
  private final SparkMax m_rightElevatorMotor = new SparkMax(MotorIDs.rightCoralElevatorMotorID, MotorType.kBrushless);
  private final RelativeEncoder m_elevatorEncoder = m_leftElevatorMotor.getEncoder();
  /** Creates a new CoralSubsystem. */
  public CoralSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(m_wristMotor.getPosition().getValueAsDouble());
  }

  public void setIntakePercentOutput(double speed){
    m_coralMotor.set(speed);
  }

  public double getWristPos(){
    return m_wristAbsoluteEncoder.getPosition() * Math.PI;
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
}
