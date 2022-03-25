// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.aTESTING;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter_SUB;

public class TestingSpinFlywheel_CMD extends CommandBase {
  private final Shooter_SUB shooter;
  double s_rpm;
  /** Creates a new SpinFlywheel_CMD. */
  public TestingSpinFlywheel_CMD(Shooter_SUB m_shooter, double s_rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    Shooter_SUB.testingrpm = s_rpm;
    shooter = m_shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.SetFlywheelVelocityControl(Shooter_SUB.testingrpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.SetFlywheelVelocityControl(0);
  }

  // Returns true when the command should end.
  
}
