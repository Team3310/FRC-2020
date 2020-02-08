/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.utilities.geometry.Pose2d;
import frc.robot.utilities.geometry.Rotation2d;
import frc.robot.utilities.geometry.Translation2d;
import frc.robot.subsystems.Limelight.LimelightConstants;
import frc.robot.subsystems.ServoMotorSubsystem.ServoMotorSubsystemConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double kLooperDt = 0.01;

    // limelight
    public static final double kHorizontalFOV = 59.6; // degrees
    public static final double kVerticalFOV = 49.7; // degrees
    public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
    public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
    public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds

    public static final double kMaxTrackerDistance = 9.0;
    public static final double kMaxGoalTrackAge = 2.5;
    public static final double kMaxGoalTrackAgeNotTracking = 0.1;
    public static final double kMaxGoalTrackSmoothingTime = 0.5;
    public static final double kTrackStabilityWeight = 0.0;
    public static final double kTrackAgeWeight = 10.0;
    public static final double kTrackSwitchingWeight = 100.0;

   

    public static final double kCameraFrameRate = 90.0;
    public static final double kMinStability = 0.5;
    public static final int kPortPipeline = 0;
    public static final int kBallPipeline = 2;
    public static final double kPortTargetHeight = 39.125;
    public static final double kHatchTargetHeight = 31.5;

    public static final double kTurretToArmOffset = -2.5;  // in
    public static final double kWristToTremorsEnd = 15.75;  // in

    // turret
    public static final ServoMotorSubsystemConstants kTurretConstants = new ServoMotorSubsystemConstants();
    static {
        kTurretConstants.kName = "Turret";

        kTurretConstants.kMasterConstants.id = 10;
        kTurretConstants.kMasterConstants.invert_motor = false;
        kTurretConstants.kMasterConstants.invert_sensor_phase = true;

        // Unit == Degrees
        kTurretConstants.kHomePosition = 0.0;  // CCW degrees from forward
        kTurretConstants.kTicksPerUnitDistance = 4096.0 * 72.0 / 18.0 * 54.0 / 16.0 / 360.0;
        kTurretConstants.kKp = 2.0;
        kTurretConstants.kKi = 0;
        kTurretConstants.kKd = 10.0;
        kTurretConstants.kKf = 0.08;
        kTurretConstants.kKa = 0.0;
        kTurretConstants.kMaxIntegralAccumulator = 0;
        kTurretConstants.kIZone = 0; // Ticks
        kTurretConstants.kDeadband = 0; // Ticks

        kTurretConstants.kPositionKp = 0.35;
        kTurretConstants.kPositionKi = 0.0;
        kTurretConstants.kPositionKd = 0.0;
        kTurretConstants.kPositionKf = 0.0;
        kTurretConstants.kPositionMaxIntegralAccumulator = 0;
        kTurretConstants.kPositionIZone = 0; // Ticks
        kTurretConstants.kPositionDeadband = 0; // Ticks

        kTurretConstants.kMinUnitsLimit = -135.0;
        kTurretConstants.kMaxUnitsLimit = 315.0;

        kTurretConstants.kCruiseVelocity = 5000; // Ticks / 100ms
        kTurretConstants.kAcceleration = 16000; // Ticks / 100ms / s
        kTurretConstants.kRampRate = 0.0; // s
        kTurretConstants.kContinuousCurrentLimit = 20; // amps
        kTurretConstants.kPeakCurrentLimit = 30; // amps
        kTurretConstants.kPeakCurrentDuration = 10; // milliseconds
        kTurretConstants.kMaxVoltage = 12.0;

        kTurretConstants.kStastusFrame8UpdateRate = 50;
        kTurretConstants.kRecoverPositionOnReset = true;
    }


    public static final LimelightConstants kLimelightConstants = new LimelightConstants();
    static {
        kLimelightConstants.kName = "Limelight";
        kLimelightConstants.kTableName = "limelight";
        kLimelightConstants.kHeight = 0.0;  // inches
        kLimelightConstants.kTurretToLens = new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0.0));
        kLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(0.0);
    }



    public static final int kCANTimeoutMs = 10; // use for important on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

    
}
