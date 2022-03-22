// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter_SUB;

public class SpinFlywheelDis_CMD extends CommandBase {
  private final Shooter_SUB shooter;
  private Color colormatched;

  /** Creates a new SpinFlywheel_CMD. */
  public SpinFlywheelDis_CMD(Shooter_SUB m_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = m_shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.isMyAllianceColor(colormatched)){
    shooter.SetFlywheelVelocityControl(2000);
    }
    else {
      shooter.SpinFlywheel(.2); //FIXME fix the speed after testing
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}