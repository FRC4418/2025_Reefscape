// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Manipulators;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class ClimberSubsystem extends SubsystemBase {
  public boolean canRun = true;
  double percent = 0;

  private final SparkMax m_climberMotor = new SparkMax(MotorIDs.climberMotorID, MotorType.kBrushless);
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {

  }

  
  public double getPosition(){
    return m_climberMotor.getEncoder().getPosition();
  }

  public void setPercentSpeed(double percent){

    if(percent < 0 && getPosition() <= 0) percent = 0;
    if(percent > 0 && getPosition() >= 370) percent = 0;
    if(canRun== false) return;
    m_climberMotor.set(percent);
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("climber pos", getPosition());
    if((percent < 0 && getPosition() < 0)||(percent > 0 && getPosition() > 320) ){
      canRun = false;
    }else{
      canRun = true;
    }
    
    // System.out.println(m_encoder.getPosition());
    // This method will be called once per scheduler run
  }
}
