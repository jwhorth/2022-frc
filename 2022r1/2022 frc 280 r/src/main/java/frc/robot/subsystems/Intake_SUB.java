// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake_SUB extends SubsystemBase {
  WPI_TalonSRX intakeMotor = new WPI_TalonSRX(26);
  WPI_TalonSRX intakeDeploy = new WPI_TalonSRX(27);
  DutyCycleEncoder pivotPositionEncoder = new DutyCycleEncoder(1);

  




  /** Creates a new SUB_PickUp. */
  public Intake_SUB() {
    
  }

  public void SetIntakeRollerspeed(double speed) {
    intakeMotor.set(speed);
  }

  public void setIntakeDeploySpeed(double speed) {
    intakeDeploy.set(speed);
  }

  public double getPivotDeployPosition() {
    return pivotPositionEncoder.get();
  }

 


  
  
 


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Encoder Position", getPivotDeployPosition());


    

  }
}
