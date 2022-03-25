// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb_SUB extends SubsystemBase {
  WPI_TalonFX climbMotor = new WPI_TalonFX(31);
  /** Creates a new SUB_Climb. */
  public Climb_SUB() {}

  public void climbMotor(double speed){
    climbMotor.set(speed);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
