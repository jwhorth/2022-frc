// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Index_SUB;
import frc.robot.subsystems.Shooter_SUB;

public class FeedBall_CMD extends CommandBase {
  private final Shooter_SUB shooter;
  private final Index_SUB index;
  /** Creates a new FeedBall_CMD. */
  public FeedBall_CMD(Shooter_SUB m_shooter, Index_SUB m_index) {
    shooter = m_shooter;
    index = m_index;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.feedMotorSpeed(-1);
    index.SetIndexRollerspeed(1);
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    /*
    
    if(shooter.readyToFire == true){
      shooter.feedMotorSpeed(.5);
      Shooter_SUB.ballCount --;
      }
      else{
        shooter.feedMotorSpeed(0);
      }

      index.SetIndexRollerspeed(.5);
      */
    }
  
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.feedMotorSpeed(0);
    
  }

  
  
}
