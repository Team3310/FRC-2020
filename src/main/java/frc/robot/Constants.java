/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.utilities.geometry.Rotation2d;
import frc.utilities.geometry.Translation2d;
import frc.robot.subsystems.Limelight.LimelightConstants;
import frc.robot.subsystems.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import frc.utilities.geometry.Pose2d;

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

    // Wheels
    // 2019 Robot Values
    public static final double kDriveWheelTrackWidthInches = 28.00; // 22.61;
    public static final double kDriveWheelDiameterInches = 3.922; // 3.875
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 0.9; // 0.924; // Tune me!

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

    // Tuned dynamics
    public static final double kRobotLinearInertia = 48.0; // kg TODO tune
    public static final double kRobotAngularInertia = 10.0; // kg m^2 TODO tune
    public static final double kRobotAngularDrag = 12.0; // N*m / (rad/sec) TODO tune
    public static final double kDriveVIntercept = 0.928112644250295; // V
    public static final double kDriveKv = 0.10305; // 0.14242500692715937; // V per rad/s
    public static final double kDriveKa = 0.01; // 0.011505866811140018; // V per rad/s^2

    public static final double finishedAtRocketLimeY = -6;
    public static final double finishedAtCargoLimeY = -8;
    public static final double finshedAtLoadingLimeY = -12;
    public static final double finishedAtRocketUlt = 8.5;
    public static final double finishedAtLoadingUlt = 10.75;
    public static final double finishedAtCargoUlt = 12;

      // Gearing and mechanical constants.
      public static final double kDriveDownShiftVelocity = 9.5 * 12.0; // inches per second
      public static final double kDriveDownShiftAngularVelocity = Math.PI / 2.0; // rad/sec
      public static final double kDriveUpShiftVelocity = 11.0 * 12.0; // inches per second
  
      public static final double kPathKX = 4.0;// 4.0; // units/s per unit of error
      public static final double kPathLookaheadTime = 0.4; // seconds to look ahead along the path for steering
      public static final double kPathMinLookaheadDistance = 24.0; // inches
  
      public static final double kDriveVelocityKp = 0.7; // 0.9;
      public static final double kDriveVelocityKi = 0.0;
      public static final double kDriveVelocityKd = 3.0; // 10.0;
      public static final double kDriveVelocityKf = 0.0;
      public static final int kDriveVelocityIZone = 0;
      public static final double kDriveVoltageRampRate = 0.0;
      public static double kDriveVelocityRampRate = 0.05; // 0.05; // 0.02
      public static double kDriveNominalOutput = 0.1;// 0.5 / 12.0;
      public static double kDriveMaxSetpoint = 11.0 * 12.0; // 11 fps
  
      public static final double kDriveSpinMoveKp = 0.05;
      public static final double kDriveSpinMoveKi = 0.0;
      public static final double kDriveSpinMoveKd = 0.0;
      public static final double kDriveSpinMoveKf = 0.14;
      public static final int kDriveSpinMoveIZone = 200;
      public static final double kDriveSpinMoveKa = 0.001;
      public static final double kDriveSpinMovekTurn = 0.02;

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
