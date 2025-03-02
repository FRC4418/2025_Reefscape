// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Manipulators;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ManipulatorGearRatios;
import frc.robot.Constants.MotorIDs;

public class CoralSubsystem extends SubsystemBase {
  private final TalonFX m_wristMotor = new TalonFX(MotorIDs.coralWristMotorID);

  private final TalonFX m_coralMotor = new TalonFX(MotorIDs.coralMotorID);
  // private final AbsoluteEncoder m_wristAbsoluteEncoder = m_coralMotor.getAbsoluteEncoder();

  private final SparkMax m_leftElevatorMotor = new SparkMax(MotorIDs.leftCoralElevatorMotorID, MotorType.kBrushless);
  private final SparkMax m_rightElevatorMotor = new SparkMax(MotorIDs.rightCoralElevatorMotorID, MotorType.kBrushless);

  private final SparkClosedLoopController  m_leftElevatorController = m_leftElevatorMotor.getClosedLoopController();
  private final SparkClosedLoopController  m_rightElevatorController = m_rightElevatorMotor.getClosedLoopController();

  private final RelativeEncoder m_elevatorEncoder = m_rightElevatorMotor.getEncoder();

  private final SparkMaxConfig followConfig = new SparkMaxConfig();

  private final AbsoluteEncoder m_wristEncoder = m_leftElevatorMotor.getAbsoluteEncoder();

  private boolean hasCoral = true;
  /** Creates a new CoralSubsystem. */
  public CoralSubsystem() {
    // m_leftElevatorMotor.configure(followConfig.follow(21, true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_coralMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    if(getCoralMotorCurrent() > 100) hasCoral = true;
    SmartDashboard.putBoolean("has coral", hasCoral);
    SmartDashboard.putNumber("current thing", m_coralMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("coral elevator pos", getElevatorPos());
    SmartDashboard.putNumber("coral wrist pos", getWristPos());
    SmartDashboard.putNumber("motor 20 current", m_leftElevatorMotor.getOutputCurrent());
    SmartDashboard.putNumber("motor 21 current", m_rightElevatorMotor.getOutputCurrent());
  }

  public double getCoralMotorCurrent(){
    return m_coralMotor.getStatorCurrent().getValueAsDouble();
  }

  public boolean hasCoral(){
    return hasCoral;
  }

  public void setHasCoral(boolean has){
    hasCoral = has;
  }

  public void setPosFancy(double pos){
    m_leftElevatorController.setReference(pos, ControlType.kMAXMotionPositionControl);
    m_rightElevatorController.setReference(pos, ControlType.kMAXMotionPositionControl);
  }

  public void setIntakePercentOutput(double speed){
    m_coralMotor.set(speed);
  }

  public double getWristPos(){
    double pos = m_wristEncoder.getPosition();
    return pos > 0.9 ? 0 : pos;
  }

  public void setWristPercentOutput(double speed){
    if(speed > .5) speed = .5;
    if(speed < -.5) speed = -.5;
    m_wristMotor.set(speed);
    SmartDashboard.putNumber("wrist percent", speed);
  }

  public double getElevatorPos(){
    return m_elevatorEncoder.getPosition();
  }

  public void setElevatorPercentOutput(double speed){
    m_leftElevatorMotor.set(speed);
    m_rightElevatorMotor.set(speed);
  }
}
