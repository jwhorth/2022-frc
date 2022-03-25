// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index_SUB;

public class IndexRun_CMD extends CommandBase {
  private final Index_SUB index;
  public IndexRun_CMD(Index_SUB m_index) {
    index = m_index;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    index.SetIndexRollerspeed(1); //FIXME The speed is subject to change.(FIX BEFORE USING)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    index.SetIndexRollerspeed(0);


  }

  
 
}
