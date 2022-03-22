// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commandgroups.FireCargo_CMD_G;
import frc.robot.commands.aTESTING.TestingSpinFlywheel_CMD;
import frc.robot.commands.index.IndexRun_CMD;
import frc.robot.commands.intake.IntakeDeploy_CMD;
import frc.robot.commands.intake.IntakeRetract_CMD;
import frc.robot.commands.intake.IntakeRun_CMD;
import frc.robot.commands.shooter.FeedBall_CMD;
import frc.robot.commands.shooter.LimelightLight_CMD;
import frc.robot.commands.turret.SeekHome_CMD;
import frc.robot.commands.turret.TestTurnTurretLeft_CMD;
import frc.robot.commands.turret.TestTurnTurretRight_CMD;
import frc.robot.commands.turret.TrackTarget_CMD;
import frc.robot.subsystems.Climb_SUB;
import frc.robot.subsystems.Drivetrain_SUB;
import frc.robot.subsystems.Index_SUB;
import frc.robot.subsystems.Shooter_SUB;
import frc.robot.subsystems.Intake_SUB;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain_SUB Drivetrain = new Drivetrain_SUB();
  private final Index_SUB Index = new Index_SUB();
  private final Shooter_SUB Shooter = new Shooter_SUB();
  private final Climb_SUB Climb = new Climb_SUB();
  private final Intake_SUB Intake = new Intake_SUB();
  Joystick operatorJoy = new Joystick(1);
    Joystick Joy1 = new Joystick(2);
    JoystickButton b1 = new JoystickButton(operatorJoy, 1);
    JoystickButton b2 = new JoystickButton(operatorJoy, 2);
    JoystickButton b3 = new JoystickButton(operatorJoy, 3);
    JoystickButton b4 = new JoystickButton(operatorJoy, 4);
    JoystickButton b5 = new JoystickButton(operatorJoy, 5);
    JoystickButton b6 = new JoystickButton(operatorJoy, 6);
    JoystickButton b7 = new JoystickButton(operatorJoy, 7);
    JoystickButton b8 = new JoystickButton(operatorJoy, 8);
    JoystickButton b9 = new JoystickButton(operatorJoy, 9);
    JoystickButton b10 = new JoystickButton(operatorJoy, 10);
    JoystickButton b11 = new JoystickButton(operatorJoy, 11);
    JoystickButton b12 = new JoystickButton(operatorJoy, 12);
    JoystickButton b13 = new JoystickButton(Joy1, 1);

  
  private final FireCargo_CMD_G m_autoCommand = new FireCargo_CMD_G(Shooter,Index); //FIXME exchange with real command system

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    b1.whenPressed(new TestingSpinFlywheel_CMD(Shooter, 10000)); 
    
    b2.whenPressed(new TestingSpinFlywheel_CMD(Shooter, 20000)); //FIXME

    b3.whenPressed(new IndexRun_CMD(Index));

    b4.whenPressed(new IntakeDeploy_CMD(Intake));

    b5.whenPressed(new IntakeRetract_CMD(Intake));

    b6.whenPressed(new IntakeRun_CMD(Intake));

    b7.whenPressed(new FeedBall_CMD(Shooter, Index));

    b8.whenPressed(new LimelightLight_CMD(Shooter)); //FIXME

    b9.whenPressed(new SeekHome_CMD(Shooter)); //FIXME

    b10.whenPressed(new TestTurnTurretLeft_CMD(Shooter));

    b11.whenPressed(new TestTurnTurretRight_CMD(Shooter));

    b12.whenPressed(new TrackTarget_CMD(Shooter)); //FIXME

  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand; //FIXME exchange with real auto functions
  }



  
}

  
  
  
