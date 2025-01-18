// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private final TalonFX armMaster = new TalonFX(31);
  private final TalonFX armSlave = new TalonFX(32);

  PIDController controller = new PIDController(0.1, 0, 0);

  MotorOutputConfigs configs = new MotorOutputConfigs();

  public Elevator() {

    
    armSlave.setControl(new Follower(armMaster.getDeviceID(), false));

  }

  public void setVelocity(double velocity){
    armMaster.set(controller.calculate(armMaster.getVelocity().getValueAsDouble(), velocity));
    armSlave.set(controller.calculate(armSlave.getVelocity().getValueAsDouble(), velocity));

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
