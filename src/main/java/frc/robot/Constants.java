/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

public final class Constants
{
    public static final double kLooperDt = 0.01;
    public static final double ENCODER_TICKS_PER_MOTOR_REVOLUTION = 2048.0;

    // USB Port IDs
    public static final int DRIVER_JOYSTICK_1_USB_ID = 0;
    public static final int OPERATOR_JOYSTICK_1_USB_ID = 1;

    // 2020 Drive Constants
    public static final double kWheelDiameterInches = 3.922;
    public static final double kTrackWidthInches = 27.5;

    public static final double kTrackWidthMeters = Units.inchesToMeters(kTrackWidthInches);
    public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackWidthMeters);

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 1.21;
    public static final double kvVoltSecondsPerMeter = 0.0628;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0204;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5; //8.5
    public static final double kDDriveVel = 0;


    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(7.0);
    public static final double kMaxAccelerationMetersPerSecondSquared = Math.pow(Units.feetToMeters(7.0), 2);

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2.0;
    public static final double kRamseteZeta = 0.7;

    // Motors
    public static final int DRIVETRAIN_RIGHT_MOTOR_MASTER_CAN_ID = 15;
    public static final int DRIVETRAIN_RIGHT_MOTOR_SLAVE_1_CAN_ID = 14;
    public static final int DRIVETRAIN_RIGHT_MOTOR_SLAVE_2_CAN_ID = 13;
    public static final int DRIVETRAIN_LEFT_MOTOR_MASTER_CAN_ID = 0;
    public static final int DRIVETRAIN_LEFT_MOTOR_SLAVE_1_CAN_ID = 1;
    public static final int DRIVETRAIN_LEFT_MOTOR_SLAVE_2_CAN_ID = 2;
    public static final int SHOOTER_MAIN_MOTOR_MASTER_CAN_ID = 4;
    public static final int SHOOTER_MAIN_MOTOR_SLAVE_CAN_ID = 11;
    public static final int SHOOTER_KICKER_MOTOR_CAN_ID = 12;
    public static final int SHOOTER_INTAKE_MOTOR_CAN_ID = 3;
    public static final int SHOOTER_HOOD_MOTOR_CAN_ID = 9;
    public static final int TURRET_MOTOR_CAN_ID = 10;
    public static final int MAGAZINE_MOTOR_CAN_ID = 5;
    public static final int INTAKE_MOTOR_CAN_ID = 7;

    // Gyro
    public static final int GYRO_CAN_ID = 0;

    // Pneumatics
    public static final int INTAKE_OUTER_ARM_PCM_ID = 2;
    public static final int INTAKE_INNER_ARM_PCM_ID = 1;
    public static final int CLIMB_ARM_RELEASE_PCM_ID = 4;

    // DIO
    public static final int TURRET_MAX_REV_SENSOR_DIO_ID = 0;
    public static final int TURRET_MIN_REV_SENSOR_DIO_ID = 1;

    // Turret
    public static final double TURRET_COMPETITION_HOME_POSITION_DEGREES = -180.0;
    public static final double TURRET_AUTO_HOME_POSITION_DEGREES = -214.1;
    public static final double TURRET_AUTO_ZERO_SPEED = -0.1;
    public static final double TURRET_MIN_ANGLE_DEGREES = -225.0;
    public static final double TURRET_MAX_ANGLE_DEGREES = 45.0;
    public static final double TURRET_INTAKE_ANGLE_DEGREES = -180.0;
    public static final double TURRET_CLIMB_LEVEL_1_ANGLE_DEGREES = -180.0;
    public static final double TURRET_GYRO_OFFSET_FENDER_SHOT_ANGLE_DEGREES = 0.0;
    public static final double TURRET_GYRO_OFFSET_AUTO_SHOT_ANGLE_DEGREES = 0.0;
    public static final double TURRET_GYRO_OFFSET_MEDIUM_SHOT_ANGLE_DEGREES = 14.0;
    public static final double TURRET_GYRO_OFFSET_LONG_SHOT_ANGLE_DEGREES = 8.0;

    // Hood
    public static final double HOOD_COMPETITION_HOME_POSITION_DEGREES = 0.0;
    public static final double HOOD_AUTO_HOME_POSITION_DEGREES = 0.0;
    public static final double HOOD_RETRACT_HOME_POSITION_DEGREES = 2.0;
    public static final double HOOD_AUTO_ZERO_SPEED = -0.1;
    public static final double HOOD_MIN_ANGLE_DEGREES = 0.0;
    public static final double HOOD_MAX_ANGLE_DEGREES = 55.0;
    public static final double HOOD_FENDER_ANGLE_DEGREES = 3.0;
    public static final double HOOD_AUTO_ANGLE_DEGREES = 37.0;
    public static final double HOOD_MEDIUM_ANGLE_DEGREES = 49.0;
    public static final double HOOD_LONG_ANGLE_DEGREES = 55.0;

    // Shooter
    public static final double SHOOTER_MAIN_FENDER_RPM = 2500;
    public static final double SHOOTER_MAIN_AUTO_RPM = 2500;
    public static final double SHOOTER_MAIN_MEDIUM_RPM = 3500;
    public static final double SHOOTER_MAIN_LONG_RPM = 4300;
    public static final double SHOOTER_MAIN_RPM_EPSILON = 100;

    public static final double SHOOTER_KICKER_FENDER_RPM = 2500;
    public static final double SHOOTER_KICKER_AUTO_RPM = 2500;
    public static final double SHOOTER_KICKER_MEDIUM_RPM = 3500;
    public static final double SHOOTER_KICKER_LONG_RPM = 4300;
    public static final double SHOOTER_KICKER_RPM_EPSILON = 100;

    public static final double SHOOTER_INTAKE_RPM = 3000;

    // Magazine
    public static final double MAGAZINE_INTAKE_RPM = 10;
    public static final double MAGAZINE_SHOOT_RPM = 60;
    public static final double MAGAZINE_SHOOT_AUTO_RPM = 40;
    public static final double MAGAZINE_SHOOT_AUTO_ROTATIONS_DEGREES = 360.0;
    public static final double MAGAZINE_JAM_STATOR_CURRENT = 40;
    public static final double MAGAZINE_COMPETITION_HOME_POSITION_DEGREES = -180.0;
    public static final double MAGAZINE_FORWARD_RPM = 15;
    public static final double MAGAZINE_REVERSE_RPM = -15;


    // Intake
    public static final double INTAKE_COLLECT_RPM = 1500;
    public static final double INTAKE_REVERSE_RPM = -1500;
    public static final double INTAKE_SLOW_RPM = 500;

    // Drive
    public static final double DRIVE_COMPETITION_GYRO_HOME_ANGLE_DEGREES = -180.0;

    // Vision
    public static final int LIMELIGHT_PIPELINE = 0;
    public static final double LIMELIGHT_OFFSET_FENDER_SHOT_DEGREES = 0.0;
    public static final double LIMELIGHT_OFFSET_AUTO_SHOT_DEGREES = 0.0;
    public static final double LIMELIGHT_OFFSET_MEDIUM_SHOT_DEGREES = -2.0;
    public static final double LIMELIGHT_OFFSET_LONG_SHOT_DEGREES = -1.0;

    // Climb
    public static final double CLIMB_MIN_INCHES = 0.0;
    public static final double CLIMB_MAX_INCHES = 30.0;
    public static final double CLIMB_LEVEL_1_INCHES = 10.0;
}