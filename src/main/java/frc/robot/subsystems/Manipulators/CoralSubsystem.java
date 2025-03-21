// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Manipulators;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.PIDConstants;

public class CoralSubsystem extends SubsystemBase {
  private final TalonFX m_wristMotor = new TalonFX(MotorIDs.coralWristMotorID);

  // private final TalonFX m_coralMotor = new TalonFX(MotorIDs.coralMotorID);
  private final SparkMax m_coralMotor = new SparkMax(MotorIDs.coralMotorID, MotorType.kBrushless);
  // private final AbsoluteEncoder m_wristAbsoluteEncoder = m_coralMotor.getAbsoluteEncoder();

  private final SparkMax m_leftElevatorMotor = new SparkMax(MotorIDs.leftCoralElevatorMotorID, MotorType.kBrushless);
  private final SparkMax m_rightElevatorMotor = new SparkMax(MotorIDs.rightCoralElevatorMotorID, MotorType.kBrushless);

  private final SparkClosedLoopController  m_leftElevatorController = m_leftElevatorMotor.getClosedLoopController();
  private final SparkClosedLoopController  m_rightElevatorController = m_rightElevatorMotor.getClosedLoopController();

  private final RelativeEncoder m_elevatorEncoder = m_rightElevatorMotor.getEncoder();

  private final AbsoluteEncoder m_wristEncoder = m_coralMotor.getAbsoluteEncoder();

  private PIDController elevatorPIDController = new PIDController(PIDConstants.kElevatorP, PIDConstants.kElevatorI, PIDConstants.kElevatorD);

  private PIDController wristPIDController = new PIDController(PIDConstants.kCoralWristP, PIDConstants.kCoralWristI, PIDConstants.kCoralWristD);

  private boolean hasCoral = true;

  private boolean isInfunnel = false;

  private double falconOffset = 0;

  public DigitalInput m_beamBreak = new DigitalInput(8);

  public DigitalInput m_limitSwitch = new DigitalInput(0);

  private double elevatorOffset = 0;

  /** Creates a new CoralSubsystem. */
  public CoralSubsystem() {
    resetFalconToSpark();
    // m_leftElevatorMotor.configure(followConfig.follow(21, true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    hasCoral = getLimitSwitch();
    if(getBeamBreak()) isInfunnel = true;
    if(hasCoral && isInfunnel) isInfunnel = false;
    SmartDashboard.putBoolean("has coral", hasCoral());
    SmartDashboard.putBoolean("is in funnel", isInFunnel());
    SmartDashboard.putNumber("current thing", getCoralMotorCurrent());
    SmartDashboard.putNumber("coral elevator pos", getElevatorPos());
    SmartDashboard.putNumber("wrist pos", getWristPos());
    SmartDashboard.putNumber("falcon pos", getFalconPos());
    SmartDashboard.putNumber("motor 20 current", m_leftElevatorMotor.getOutputCurrent());
    SmartDashboard.putNumber("motor 21 current", m_rightElevatorMotor.getOutputCurrent());
    SmartDashboard.putNumber("elevator speed", getElevatorSpeed());
    SmartDashboard.putBoolean("beam break", m_beamBreak.get());
    SmartDashboard.putBoolean("limit switch", m_limitSwitch.get());
    SmartDashboard.putNumber("coral intake current", getCoralMotorCurrent());
  }

  public double getCoralMotorCurrent(){
    return m_coralMotor.getOutputCurrent() * (m_coralMotor.getEncoder().getVelocity()/-5000);
  }

  public boolean hasCoral(){
    return hasCoral;
  }

  public boolean isInFunnel(){
    return isInfunnel;
  }

  public boolean getLimitSwitch(){
    return m_limitSwitch.get();
  }

  public boolean getBeamBreak(){
    return m_beamBreak.get();
  }

  public void setManipulatorPos(double elevatorPos, double wristPos){
    
    double elevatorPIDvalue = elevatorPIDController.calculate(getElevatorPos(), elevatorPos);
    double wristPIDValue = wristPIDController.calculate(getWristPos(), wristPos);
    double wristStall = Math.cos(2*Math.PI*(getWristPos()-0.13)) * (hasCoral ? PIDConstants.kCoralWristrStallMulti : PIDConstants.kNoCoralWristrStallMulti);

    setWristPercentOutput(wristStall + wristPIDValue);


    SmartDashboard.putNumber("command wrist pid out", wristPIDValue + wristStall);

    double maxElevatorPercent = getElevatorSpeed()/4000+.3;

    elevatorPIDvalue = MathUtil.clamp(elevatorPIDvalue, -maxElevatorPercent, maxElevatorPercent);

    setElevatorPercentOutput(elevatorPIDvalue);
  }

  public void setPosFancy(double pos){
    m_leftElevatorController.setReference(pos, ControlType.kMAXMotionPositionControl);
    m_rightElevatorController.setReference(pos, ControlType.kMAXMotionPositionControl);
  }

  public void setIntakePercentOutput(double speed){
    m_coralMotor.set(speed);
  }

  public double getWristPos(){
    double sparkPos = m_wristEncoder.getPosition();
    sparkPos = sparkPos > 0.9 ? 0 : sparkPos;

    double falconPos = getFalconPos();

    if(sparkPos - falconPos > 0.1 || sparkPos - falconPos < -0.1){
      // return falconPos;
      // resetFalconToSpark();
    }
    return sparkPos;
  }

  public double getFalconPos(){
    return -m_wristMotor.getPosition().getValueAsDouble()/9 + falconOffset;
  }

  public void resetFalconToSpark(){
    falconOffset = m_wristEncoder.getPosition() - getFalconPos();
  }

  public void setWristPercentOutput(double speed){
    if(speed > .5) speed = .5;
    if(speed < -.5) speed = -.5;
    m_wristMotor.set(-speed);
    SmartDashboard.putNumber("wrist percent", -speed);
  }

  public double getElevatorPos(){
    return m_elevatorEncoder.getPosition() + elevatorOffset;
  }

  public void resetElevatorPos(){
    elevatorOffset = -m_elevatorEncoder.getPosition();
  }

  public double getElevatorSpeed(){
    return Math.abs(m_elevatorEncoder.getVelocity());
  }

  public void setElevatorPercentOutput(double speed){
    m_leftElevatorMotor.set(speed);
    m_rightElevatorMotor.set(speed);
  }
}
