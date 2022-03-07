// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commandgroups.FireCargo_CMD_G;
import frc.robot.subsystems.Climb_SUB;
import frc.robot.subsystems.Drivetrain_SUB;
import frc.robot.subsystems.Index_SUB;
import frc.robot.subsystems.Shooter_SUB;
import frc.robot.subsystems.Intake_SUB;


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
  private final Intake_SUB PickUp = new Intake_SUB();

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
    button.
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
