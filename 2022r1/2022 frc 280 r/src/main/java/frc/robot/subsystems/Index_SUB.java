// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;





public class Index_SUB extends SubsystemBase {
  WPI_TalonSRX indexMotor1 = new WPI_TalonSRX(0);
  WPI_TalonSRX indexMotor2 = new WPI_TalonSRX(0);
  /** Creates a new SUB_Index. */
  public Index_SUB() {
  }  
  
  public void SetIndexRollerspeed(double speed) {
    indexMotor1.set(speed);
    indexMotor2.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
