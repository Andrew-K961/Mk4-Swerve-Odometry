// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(23.0); //  Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(23.0); //  Measure and set wheelbase

    public static final int DRIVETRAIN_PIGEON_ID = 13;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; //  Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 3; //  Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 2; //  Set front left steer encoder ID

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4; //  Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 6; //  Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 5; //  Set front right steer encoder ID

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 10; //  Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 12; //  Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11; //  Set back left steer encoder ID

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7; //  Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 9; //  Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 8; //  Set back right steer encoder ID

    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(255);
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(196.7 - 180);
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(239.3);
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(156 + 180);
}
