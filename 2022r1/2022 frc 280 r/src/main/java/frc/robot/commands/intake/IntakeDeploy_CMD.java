// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake_SUB;

public class IntakeDeploy_CMD extends CommandBase {
  private final Intake_SUB intake;
  
  public IntakeDeploy_CMD(Intake_SUB m_intake) {
    intake = m_intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakeDeploySpeed(.5); //FIXME The speed is subject to change.(FIX BEFORE USING)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakeDeploySpeed(0);
  }

  // Returns true when the command should end.
  
}
