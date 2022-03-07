// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.shooter.FeedBall_CMD;
import frc.robot.commands.shooter.SpinFlywheelDis_CMD;
import frc.robot.commands.shooter.StopFlywheel_CMD;
import frc.robot.subsystems.Index_SUB;
import frc.robot.subsystems.Shooter_SUB;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FireCargo_CMD_G extends SequentialCommandGroup {
  /** Creates a new FireCargo_CMD_G. */
  public FireCargo_CMD_G(Shooter_SUB shooter, Index_SUB index) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SpinFlywheelDis_CMD(shooter),
      new FeedBall_CMD(shooter,index),
      new StopFlywheel_CMD(shooter)
    );
  }
}
