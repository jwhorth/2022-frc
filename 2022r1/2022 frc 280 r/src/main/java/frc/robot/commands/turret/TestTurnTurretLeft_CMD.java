// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter_SUB;

public class TestTurnTurretLeft_CMD extends CommandBase {
  private final Shooter_SUB shooter;
  /** Creates a new TestTurnTurretRight_CMD. */
  public TestTurnTurretLeft_CMD(Shooter_SUB m_shooter) {
    shooter = m_shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.spinTurretMotor(-0.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.spinTurretMotor(0);
  }

  // Returns true when the command should end.
 
  
}

