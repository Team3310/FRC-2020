/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public final class Constants
{
    public static final double kLooperDt = 0.01;
    public static final double ENCODER_TICKS_PER_MOTOR_REVOLUTION = 2048.0;

    // USB Port IDs
    public static final int DRIVER_JOYSTICK_1_USB_ID = 0;
    public static final int OPERATOR_JOYSTICK_1_USB_ID = 1;

    // Motors
    public static final int DRIVETRAIN_RIGHT_MOTOR_MASTER_CAN_ID = 0;
    public static final int DRIVETRAIN_RIGHT_MOTOR_SLAVE_1_CAN_ID = 1;
    public static final int DRIVETRAIN_RIGHT_MOTOR_SLAVE_2_CAN_ID = 2;
    public static final int DRIVETRAIN_LEFT_MOTOR_MASTER_CAN_ID = 15;
    public static final int DRIVETRAIN_LEFT_MOTOR_SLAVE_1_CAN_ID = 14;
    public static final int DRIVETRAIN_LEFT_MOTOR_SLAVE_2_CAN_ID = 13;
    public static final int SHOOTER_MAIN_MOTOR_MASTER_CAN_ID = 15;
    public static final int SHOOTER_MAIN_MOTOR_SLAVE_CAN_ID = 15;
    public static final int SHOOTER_KICKER_MOTOR_CAN_ID = 15;
    public static final int SHOOTER_INTAKE_MOTOR_CAN_ID = 15;
    public static final int SHOOTER_HOOD_MOTOR_CAN_ID = 15;
    public static final int TURRET_MOTOR_CAN_ID = 3;
    public static final int MAGAZINE_MOTOR_CAN_ID = 12;
    public static final int INTAKE_MOTOR_CAN_ID = 10;
    public static final int CLIMB_MOTOR_CAN_ID = 9;

    // Pneumatics
    public static final int INTAKE_OUTER_ARM_PCM_ID = 1;
    public static final int INTAKE_INNER_ARM_PCM_ID = 2;
    public static final int CLIMB_MOTOR_PTO_PCM_ID = 3;
    public static final int CLIMB_ARM_RELEASE_PCM_ID = 4;
    public static final int BUDDY_LEG_RELEASE_PCM_ID = 5;

    // DIO
    public static final int TURRET_MAX_REV_SENSOR_DIO_ID = 8;
    public static final int TURRET_MIN_REV_SENSOR_DIO_ID = 9;

    // Turret
    public static final double TURRET_AUTO_HOME_POSITION_DEGREES = -45.0;
    public static final double TURRET_AUTO_ZERO_SPEED = -0.1;
    public static final double TURRET_MIN_ANGLE_DEGREES = -45.0;
    public static final double  TURRET_MAX_ANGLE_DEGREES = 270.0;
}