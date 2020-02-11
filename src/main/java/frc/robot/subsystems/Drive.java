/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.planners.DriveMotionPlanner;
import frc.robot.subsystems.ServoMotorSubsystem.PeriodicIO;
import frc.utilities.geometry.Pose2d;
import frc.utilities.geometry.Pose2dWithCurvature;
import frc.utilities.geometry.Rotation2d;
import frc.utilities.trajectory.timing.TimedState;
import frc.utilities.util.ReflectingCSVWriter;

/**
 * Add your docs here.
 */
public class Drive extends Subsystem {
  private static Drive instance;

	public static enum DriveControlMode {
		JOYSTICK, MP_STRAIGHT, MP_TURN, PID_TURN, HOLD, MANUAL, VELOCITY_SETPOINT, CAMERA_TRACK, PATH_FOLLOWING,
		OPEN_LOOP, CAMERA_TRACK_DRIVE, SPIN_MOVE
  };
  
  public static Drive getInstance() {
		if (instance == null) {
			instance = new Drive();
		}
		return instance;
  }
  
  public synchronized Rotation2d getHeading() {
		return mPeriodicIO.gyro_heading;
	}

	public synchronized void setHeading(Rotation2d heading) {
		System.out.println("SET HEADING: " + heading.getDegrees());

		mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(gyroPigeon.getFusedHeading()).inverse());
		System.out.println("Gyro offset: " + mGyroOffset.getDegrees());

		mPeriodicIO.gyro_heading = heading;
  }
  
  // Hardware states //Poofs
	private PeriodicIO mPeriodicIO;
	private SpinMoveIO mSpinMoveIO;
	private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
	private DriveMotionPlanner mMotionPlanner;
	private Rotation2d mGyroOffset = Rotation2d.identity();
  public boolean mOverrideTrajectory = false;
  
  public static class SpinMoveIO {
		public double totalTime;
		public double deltaTime;
		public double targetTime;
		public double targetTheta;
		public double actualTheta;
		public double targetLeftVelocity;
		public double actualLeftVelocity;
		public double targetRightVelocity;
		public double actualRightVelocity;
		public double targetRightAcceleration;
		public double targetLeftAcceleration;
  }

  public static class PeriodicIO {
		// INPUTS
		public int left_position_ticks;
		public int right_position_ticks;
		public double left_distance;
		public double right_distance;
		public int left_velocity_ticks_per_100ms;
		public int right_velocity_ticks_per_100ms;
		public Rotation2d gyro_heading = Rotation2d.identity();
		public Pose2d error = Pose2d.identity();

		// OUTPUTS
		public double left_demand;
		public double right_demand;
		public double left_accel;
		public double right_accel;
		public double left_feedforward;
		public double right_feedforward;
		public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(
		Pose2dWithCurvature.identity());
  }
  
  private PigeonIMU gyroPigeon;
	private double[] yprPigeon = new double[3];
	private short[] xyzPigeon = new short[3];
	private boolean useGyroLock;
	private double gyroLockAngleDeg;
	private double kPGyro = 0.04;
	private boolean isCalibrating = false;
	private double gyroOffsetDeg = 0;

  @Override
  public void stop() {
    // TODO Auto-generated method stub

  }

  @Override
  public boolean checkSystem() {
    // TODO Auto-generated method stub
    return false;
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub

  }
  
 
  
}
