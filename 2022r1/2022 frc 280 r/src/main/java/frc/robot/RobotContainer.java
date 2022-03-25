// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commandgroups.FireCargo_CMD_G;
import frc.robot.commandgroups.PickupCargo_CMD_G;
import frc.robot.commands.aTESTING.TestingSpinFlywheel_CMD;
import frc.robot.commands.drive.DefaultDriveCommand;
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
import frc.robot.subsystems.DrivetrainSubsystem;
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
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final Index_SUB Index = new Index_SUB();
  private final Shooter_SUB Shooter = new Shooter_SUB();
  private final Climb_SUB Climb = new Climb_SUB();
  private final Intake_SUB Intake = new Intake_SUB();
  public double ballCount;
  Joystick driveMovement = new Joystick(0);
  Joystick driveRotation = new Joystick(1);
  Joystick operatorJoy = new Joystick(2);
  Joystick driverButtons = new Joystick(3);
    
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
    JoystickButton dB1 = new JoystickButton(driverButtons, 1);
    

  
  private final FireCargo_CMD_G m_autoCommand = new FireCargo_CMD_G(Shooter,Index); //FIXME exchange with real command system

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
      m_drivetrainSubsystem,
      () -> -modifyAxis(driveMovement.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(driveMovement.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(driveRotation.getX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
));

    

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
    
    b1.toggleWhenPressed(new PickupCargo_CMD_G(Index, Intake));
    
    b2.toggleWhenPressed(new TestingSpinFlywheel_CMD(Shooter,9700)); //FIXME

    b3.whenHeld(new FeedBall_CMD(Shooter, Index));


    b4.whenHeld(new IntakeDeploy_CMD(Intake));

    b5.whenHeld(new IntakeRetract_CMD(Intake));

    b8.whenPressed(new LimelightLight_CMD(Shooter)); //FIXME

    b9.whenHeld(new TestTurnTurretLeft_CMD(Shooter));

    b10.whenHeld(new TestTurnTurretRight_CMD(Shooter));
    
    dB1.whenPressed(m_drivetrainSubsystem::zeroGyroscope);

   

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
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }


  
}

  
  
  
