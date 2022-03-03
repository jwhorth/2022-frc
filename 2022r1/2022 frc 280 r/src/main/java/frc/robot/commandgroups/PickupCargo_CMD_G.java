// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.index.IndexRun_CMD;
import frc.robot.commands.intake.IntakeRun_CMD;
import frc.robot.subsystems.Index_SUB;
import frc.robot.subsystems.Intake_SUB;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickupCargo_CMD_G extends ParallelCommandGroup {
  /** Creates a new PickupCargo_CMDGRP. */
  public PickupCargo_CMD_G(Index_SUB index, Intake_SUB intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IndexRun_CMD(index), 
      new IntakeRun_CMD(intake)
      );
  }
}
