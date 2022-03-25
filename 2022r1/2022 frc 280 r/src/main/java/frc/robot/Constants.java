// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //FIXME These need to be changed when can id's are figured out.
    //Index
    public static final int Index_Motor_CANid = 25;
    public static final int Index_IR_DIOid = 0;
    //Intake
    public static final int Intake_Motor_CANid = 26;
    public static final int Intake_Deploy_CANid = 27;
    //Shooter
    public static final int Shooter_Motor_CANid = 22;
    public static final int Feed_Motor_CANid = 24;
    public static final int Rotation_Motor_CANid = 23;
    //Climber

    //Drive
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.5715; // Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4953; // Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 0; //  Set Pigeon ID

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 11; // Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 12; // Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 1; //Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(137.45269775390625); // FIXME Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 13; // Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 14; // Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 2; //Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(24.609375); // FIXME Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 15; // Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 16; // Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 3; //Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(31.7230224609375); // FIXME Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 17; // Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 18; // Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 4; //Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(332.13043212890625); // FIXME Measure and set back right steer offset



    public static final ShuffleboardTab SORTER_TAB = Shuffleboard.getTab("Sorter");
    public static final ShuffleboardTab PRIMARY_TAB = Shuffleboard.getTab("Primary");
}
