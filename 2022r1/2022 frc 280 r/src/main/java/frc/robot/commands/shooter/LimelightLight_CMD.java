// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter_SUB;

public class LimelightLight_CMD extends CommandBase {
  private final Shooter_SUB shooter;
  /** Creates a new LimelightLight_CMD. */
  public LimelightLight_CMD(Shooter_SUB m_shooter) {
    shooter = m_shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.table.getEntry("ledMode").setNumber(3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.table.getEntry("ledMode").setNumber(1);
  }

  // Returns true when the command should end.
 
}
