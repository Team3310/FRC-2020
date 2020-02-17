/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.wpilibj.Ultrasonic;
import frc.robot.Constants;
import frc.utilities.drivers.MPSoftwarePIDController;
import frc.utilities.drivers.MPTalonPIDController;
import frc.utilities.drivers.SoftwarePIDController;
import frc.utilities.drivers.TalonFXEncoder;
import frc.utilities.drivers.MPSoftwarePIDController.MPSoftwareTurnType;
import frc.utilities.geometry.Pose2d;
import frc.utilities.geometry.Pose2dWithCurvature;
import frc.utilities.geometry.Rotation2d;
import frc.utilities.trajectory.timing.TimedState;
import frc.utilities.util.BHRMathUtils;
import frc.utilities.util.ReflectingCSVWriter;

/**
 * Add your docs here.
 */
public class Drive extends Subsystem {
	private static Drive instance;

	public static enum DriveControlMode {
	  JOYSTICK, MP_STRAIGHT, MP_TURN, PID_TURN, HOLD, MANUAL, VELOCITY_SETPOINT, CAMERA_TRACK, PATH_FOLLOWING, OPEN_LOOP,
	  CAMERA_TRACK_DRIVE, SPIN_MOVE
	};
  
	// Speed Control
	private static final double DRIVE_OUTPUT_TO_ENCODER_RATIO = 48.0 / 48.0;
	private static final double TICKS_PER_ROTATION = 2048.0;
	public static final double TRACK_WIDTH_INCHES = 0.0;
	private static final double DRIVE_ENCODER_PPR = 4096.;
  
	public static final double MP_STRAIGHT_T1 = 600;
	public static final double MP_STRAIGHT_T2 = 300;
	public static final double MP_TURN_T1 = 600;
	public static final double MP_TURN_T2 = 300;
	public static final double MP_MAX_TURN_T1 = 200;
	public static final double MP_MAX_TURN_T2 = 100;
  
	public static final double OPEN_LOOP_VOLTAGE_RAMP_HI = 0.0;
	public static final double OPEN_LOOP_VOLTAGE_RAMP_LO = 0.1;
  
	// Motor Controllers
	private ArrayList<TalonFXEncoder> motorControllers = new ArrayList<TalonFXEncoder>();
  
	// Left Drive
	private TalonFX leftDrive1;
	private TalonFX leftDrive2;
	private TalonFX leftDrive3;
  
	// Right Drive
	private TalonFX rightDrive1;
	private TalonFX rightDrive2;
	private TalonFX rightDrive3;
  
	public Drive() {
	  leftDrive1 = new TalonFX(0);
	  leftDrive2 = new TalonFX(1);
	  leftDrive3 = new TalonFX(2);
  
	  rightDrive1 = new TalonFX(3);
	  rightDrive2 = new TalonFX(4);
	  rightDrive3 = new TalonFX(5);
  
	  leftDrive1.configFactoryDefault();
	  leftDrive2.configFactoryDefault();
	  leftDrive3.configFactoryDefault();
  
	  rightDrive1.configFactoryDefault();
	  rightDrive2.configFactoryDefault();
	  rightDrive3.configFactoryDefault();
  
	  SupplyCurrentLimitConfiguration supplyCurrentConfigs = new SupplyCurrentLimitConfiguration();
	  supplyCurrentConfigs.currentLimit = 30;
	  supplyCurrentConfigs.enable = true;
  
	  leftDrive1.configSupplyCurrentLimit(supplyCurrentConfigs);
	  leftDrive2.configSupplyCurrentLimit(supplyCurrentConfigs);
	  leftDrive3.configSupplyCurrentLimit(supplyCurrentConfigs);
  
	  rightDrive1.configSupplyCurrentLimit(supplyCurrentConfigs);
	  rightDrive2.configSupplyCurrentLimit(supplyCurrentConfigs);
	  rightDrive3.configSupplyCurrentLimit(supplyCurrentConfigs);
  
	  // k constants
	  leftDrive1.config_kF(0, 0.053);
	  leftDrive1.config_kP(0, 0.50);
	  leftDrive1.config_kI(0, 0.00001);
	  leftDrive1.config_kD(0, 0.0);
  
	  leftDrive2.config_kF(0, 0.053);
	  leftDrive2.config_kP(0, 0.50);
	  leftDrive2.config_kI(0, 0.00001);
	  leftDrive2.config_kD(0, 0.0);
  
	  leftDrive3.config_kF(0, 0.053);
	  leftDrive3.config_kP(0, 0.50);
	  leftDrive3.config_kI(0, 0.00001);
	  leftDrive3.config_kD(0, 0.0);
  
	  rightDrive1.config_kF(0, 0.053);
	  rightDrive1.config_kP(0, 0.50);
	  rightDrive1.config_kI(0, 0.00001);
	  rightDrive1.config_kD(0, 0.0);
  
	  rightDrive2.config_kF(0, 0.053);
	  rightDrive2.config_kP(0, 0.50);
	  rightDrive2.config_kI(0, 0.00001);
	  rightDrive2.config_kD(0, 0.0);
  
	  rightDrive3.config_kF(0, 0.053);
	  rightDrive3.config_kP(0, 0.50);
	  rightDrive3.config_kI(0, 0.00001);
	  rightDrive3.config_kD(0, 0.0);
  
	  try {
		leftDrive1.setSensorPhase(false);
		leftDrive2.setSensorPhase(false);
		leftDrive3.setSensorPhase(false);
  
		leftDrive1.setInverted(true);
		leftDrive2.setInverted(true);
		leftDrive3.setInverted(true);
  
		rightDrive1.setSensorPhase(false);
		rightDrive2.setSensorPhase(false);
		rightDrive3.setSensorPhase(false);
  
		rightDrive1.setInverted(false);
		rightDrive2.setInverted(false);
		rightDrive3.setInverted(false);
  
		configureMaster(leftDrive1);
		configureMaster(leftDrive2);
		configureMaster(leftDrive3);
		configureMaster(rightDrive1);
		configureMaster(rightDrive2);
		configureMaster(rightDrive3);
  
		motorControllers.add((TalonFXEncoder) leftDrive1);
		motorControllers.add((TalonFXEncoder) leftDrive2);
		motorControllers.add((TalonFXEncoder) leftDrive3);
		motorControllers.add((TalonFXEncoder) rightDrive1);
		motorControllers.add((TalonFXEncoder) rightDrive2);
		motorControllers.add((TalonFXEncoder) rightDrive3);
  
		setBrakeMode(true);
  
	  } catch (Exception e) {
		System.err.println("An error occurred in the DriveTrain constructor");
	  }
  
	}
  
	public boolean isBrakeMode() {
	  return mIsBrakeMode;
	}
  
	public synchronized void setBrakeMode(boolean on) {
	  if (mIsBrakeMode != on) {
		mIsBrakeMode = on;
		rightDrive1.setNeutralMode(NeutralMode.Brake);
		rightDrive2.setNeutralMode(NeutralMode.Brake);
		rightDrive3.setNeutralMode(NeutralMode.Brake);
		leftDrive1.setNeutralMode(NeutralMode.Brake);
		leftDrive2.setNeutralMode(NeutralMode.Brake);
		leftDrive3.setNeutralMode(NeutralMode.Brake);
	  }
	}
  
	// PID Parameters
	public class PIDParams {
	  public double kP = 0;
	  public double kI = 0;
	  public double kD = 0;
	  public double kF = 0;
	  public double kA = 0;
	  public double kV = 0;
	  public double kG = 0;
	  public double iZone = 0;
	  public double rampRatePID = 0;
  
	  public PIDParams() {
	  }
  
	  public PIDParams(double kP, double kI, double kD) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
	  }
  
	  public PIDParams(double kP, double kI, double kD, double kF) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
	  }
  
	  public PIDParams(double kP, double kI, double kD, double kA, double kV) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kA = kA;
		this.kV = kV;
	  }
  
	  public PIDParams(double kP, double kI, double kD, double kA, double kV, double kG) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kA = kA;
		this.kV = kV;
		this.kG = kG;
	  }
  
	  public PIDParams(double kP, double kI, double kD, double kA, double kV, double kG, double iZone) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kA = kA;
		this.kV = kV;
		this.kG = kG;
		this.iZone = iZone;
	  }
	}
  
	private DriveControlMode driveControlMode = DriveControlMode.JOYSTICK;

	private boolean isFinished;
  
	private static final int kPositionControlSlot = 0;
	private static final int kVelocityControlSlot = 1;
	private static final int kSpinMoveControlSlot = 2;
  
	protected Rotation2d mAngleAdjustment = new Rotation2d();
  
	private boolean isRed = true;
	private boolean mIsBrakeMode = false;
  
	private MPTalonPIDController mpStraightController;
	private PIDParams mpStraightPIDParams = new PIDParams(0.1, 0, 0, 0.005, 0.03, 0.15);
	private PIDParams mpHoldPIDParams = new PIDParams(1, 0, 0, 0.0, 0.0, 0.0);
  
	private MPSoftwarePIDController mpTurnController; // p i d a v g izone
	private PIDParams mpTurnPIDParams = new PIDParams(0.005, 0.0001, 0.2, 0.00035, 0.0025, 0.0, 100);
  
	private SoftwarePIDController pidTurnController;
	private PIDParams pidTurnPIDParams = new PIDParams(0.04, 0.001, 0.4, 0, 0, 0.0, 100); // i=0.0008
  
	private PigeonIMU gyroPigeon;
	private double[] yprPigeon = new double[3];
	private short[] xyzPigeon = new short[3];
	private boolean useGyroLock;
	private double gyroLockAngleDeg;
	private double kPGyro = 0.04;
	private boolean isCalibrating = false;
	private double gyroOffsetDeg = 0;
  
	private double mLastValidGyroAngle;
	private double mCameraVelocity = 0;
	private double mCameraSetVelocity = 0;
	private double kCamera = 0.0475; // .7
	private double kCameraDriveClose = 0.08; // .072
	private double kCameraDriveMid = 0.043; // .04
	private double kCameraDriveFar = 0.033; // .04
	private double kCameraClose = 10;
	private double kCameraMid = 15;
	private double kCameraFar = 20;
  
	// Hardware states //Poofs
	private PeriodicIO mPeriodicIO;
	private SpinMoveIO mSpinMoveIO;
	private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
	private Rotation2d mGyroOffset = new Rotation2d();
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
		  public Rotation2d gyro_heading = new Rotation2d();
		  public Pose2d error = new Pose2d();
  
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
  
	
	// Vision
	  public double limeArea;
	  public double lastValidLimeArea;
	  public double limeX;
	  public double limeY;
	  public double limeSkew;
	  public boolean isLimeValid;
	  public double LEDMode;
	  public double camMode;
	  public boolean onTarget;
  
	// Ultrasonic
	  public Ultrasonic ultrasonic = new Ultrasonic(4, 5);
  
	  public double targetDrivePositionTicks;
	  public double targetMiddlePositionTicks;
	  public double targetSpinAngle;
	  public double spinMoveStartAngle;
	  public double spinMoveStartVelocity;
	private double lastTime = 0;
  
	// Configures Falcons for Velocity Control
	public void configureFalconsForSpeedControl() {
	  if(!usesFalconVelocityControl(driveControlMode)) {
		leftDrive1.enableVoltageCompensation(true);
		leftDrive1.configVoltageCompSaturation(12.0, TalonFXEncoder.TIMEOUT_MS);
		leftDrive1.configPeakOutputForward(+1.0f, TalonFXEncoder.TIMEOUT_MS);
		leftDrive1.configPeakOutputReverse(-1.0f, TalonFXEncoder.TIMEOUT_MS);
  
		leftDrive2.enableVoltageCompensation(true);
		leftDrive2.configVoltageCompSaturation(12.0, TalonFXEncoder.TIMEOUT_MS);
		leftDrive2.configPeakOutputForward(+1.0f, TalonFXEncoder.TIMEOUT_MS);
		leftDrive2.configPeakOutputReverse(-1.0f, TalonFXEncoder.TIMEOUT_MS);
  
		leftDrive3.enableVoltageCompensation(true);
		leftDrive3.configVoltageCompSaturation(12.0, TalonFXEncoder.TIMEOUT_MS);
		leftDrive3.configPeakOutputForward(+1.0f, TalonFXEncoder.TIMEOUT_MS);
		leftDrive3.configPeakOutputReverse(-1.0f, TalonFXEncoder.TIMEOUT_MS);
  
		rightDrive1.enableVoltageCompensation(true);
		rightDrive1.configVoltageCompSaturation(12.0, TalonFXEncoder.TIMEOUT_MS);
		rightDrive1.configPeakOutputForward(+1.0f, TalonFXEncoder.TIMEOUT_MS);
		rightDrive1.configPeakOutputReverse(-1.0f, TalonFXEncoder.TIMEOUT_MS);
  
		rightDrive2.enableVoltageCompensation(true);
		rightDrive2.configVoltageCompSaturation(12.0, TalonFXEncoder.TIMEOUT_MS);
		rightDrive2.configPeakOutputForward(+1.0f, TalonFXEncoder.TIMEOUT_MS);
		rightDrive2.configPeakOutputReverse(-1.0f, TalonFXEncoder.TIMEOUT_MS);
  
		rightDrive3.enableVoltageCompensation(true);
		rightDrive3.configVoltageCompSaturation(12.0, TalonFXEncoder.TIMEOUT_MS);
		rightDrive3.configPeakOutputForward(+1.0f, TalonFXEncoder.TIMEOUT_MS);
		rightDrive3.configPeakOutputReverse(-1.0f, TalonFXEncoder.TIMEOUT_MS);
  
		System.out.println("configureTalonsForSpeedControl");
			  leftDrive1.selectProfileSlot(kVelocityControlSlot, TalonFXEncoder.PID_IDX);
			  leftDrive1.configNominalOutputForward(Constants.kDriveNominalOutput, TalonFXEncoder.TIMEOUT_MS);
			  leftDrive1.configNominalOutputReverse(-Constants.kDriveNominalOutput, TalonFXEncoder.TIMEOUT_MS);
		leftDrive1.configClosedloopRamp(Constants.kDriveVelocityRampRate, TalonFXEncoder.TIMEOUT_MS);
		
		leftDrive2.selectProfileSlot(kVelocityControlSlot, TalonFXEncoder.PID_IDX);
			  leftDrive2.configNominalOutputForward(Constants.kDriveNominalOutput, TalonFXEncoder.TIMEOUT_MS);
			  leftDrive2.configNominalOutputReverse(-Constants.kDriveNominalOutput, TalonFXEncoder.TIMEOUT_MS);
			  leftDrive2.configClosedloopRamp(Constants.kDriveVelocityRampRate, TalonFXEncoder.TIMEOUT_MS);
  
		leftDrive3.selectProfileSlot(kVelocityControlSlot, TalonFXEncoder.PID_IDX);
			  leftDrive3.configNominalOutputForward(Constants.kDriveNominalOutput, TalonFXEncoder.TIMEOUT_MS);
			  leftDrive3.configNominalOutputReverse(-Constants.kDriveNominalOutput, TalonFXEncoder.TIMEOUT_MS);
			  leftDrive3.configClosedloopRamp(Constants.kDriveVelocityRampRate, TalonFXEncoder.TIMEOUT_MS);
  
		rightDrive1.selectProfileSlot(kVelocityControlSlot, TalonFXEncoder.PID_IDX);
			  rightDrive1.configNominalOutputForward(Constants.kDriveNominalOutput, TalonFXEncoder.TIMEOUT_MS);
			  rightDrive1.configNominalOutputReverse(-Constants.kDriveNominalOutput, TalonFXEncoder.TIMEOUT_MS);
		rightDrive1.configClosedloopRamp(Constants.kDriveVelocityRampRate, TalonFXEncoder.TIMEOUT_MS);
		
		rightDrive2.selectProfileSlot(kVelocityControlSlot, TalonFXEncoder.PID_IDX);
			  rightDrive2.configNominalOutputForward(Constants.kDriveNominalOutput, TalonFXEncoder.TIMEOUT_MS);
			  rightDrive2.configNominalOutputReverse(-Constants.kDriveNominalOutput, TalonFXEncoder.TIMEOUT_MS);
		rightDrive2.configClosedloopRamp(Constants.kDriveVelocityRampRate, TalonFXEncoder.TIMEOUT_MS);
		
		rightDrive3.selectProfileSlot(kVelocityControlSlot, TalonFXEncoder.PID_IDX);
			  rightDrive3.configNominalOutputForward(Constants.kDriveNominalOutput, TalonFXEncoder.TIMEOUT_MS);
			  rightDrive3.configNominalOutputReverse(-Constants.kDriveNominalOutput, TalonFXEncoder.TIMEOUT_MS);
			  rightDrive3.configClosedloopRamp(Constants.kDriveVelocityRampRate, TalonFXEncoder.TIMEOUT_MS);
	  }
	}

	protected static boolean usesFalconVelocityControl(DriveControlMode state) {
		if (state == DriveControlMode.VELOCITY_SETPOINT || state == DriveControlMode.PATH_FOLLOWING
				|| state == DriveControlMode.CAMERA_TRACK || state == DriveControlMode.SPIN_MOVE) {
			return true;
		}
		return false;
	}

  
	private void configureMaster(TalonFX talon) {
		  talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
		  talon.enableVoltageCompensation(true);
		  talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
		  talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
		  talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
		  talon.configClosedloopRamp(Constants.kDriveVoltageRampRate, Constants.kLongCANTimeoutMs);
		  talon.configNeutralDeadband(0.04, 0);
	}
  
	// Spin Move
	private void configSpinMove() {
		  // Set PID gains for spin move
		  rightDrive1.config_kP(kSpinMoveControlSlot, Constants.kDriveSpinMoveKp, Constants.kLongCANTimeoutMs);
		  rightDrive1.config_kI(kSpinMoveControlSlot, Constants.kDriveSpinMoveKi, Constants.kLongCANTimeoutMs);
		  rightDrive1.config_kD(kSpinMoveControlSlot, Constants.kDriveSpinMoveKd, Constants.kLongCANTimeoutMs);
		  rightDrive1.config_kF(kSpinMoveControlSlot, Constants.kDriveSpinMoveKf, Constants.kLongCANTimeoutMs);
		  rightDrive1.config_IntegralZone(kSpinMoveControlSlot, Constants.kDriveSpinMoveIZone,
		  Constants.kLongCANTimeoutMs);
	  
	  rightDrive2.config_kP(kSpinMoveControlSlot, Constants.kDriveSpinMoveKp, Constants.kLongCANTimeoutMs);
	  rightDrive2.config_kI(kSpinMoveControlSlot, Constants.kDriveSpinMoveKi, Constants.kLongCANTimeoutMs);
	  rightDrive2.config_kD(kSpinMoveControlSlot, Constants.kDriveSpinMoveKd, Constants.kLongCANTimeoutMs);
	  rightDrive2.config_kF(kSpinMoveControlSlot, Constants.kDriveSpinMoveKf, Constants.kLongCANTimeoutMs);
	  rightDrive2.config_IntegralZone(kSpinMoveControlSlot, Constants.kDriveSpinMoveIZone,
		  Constants.kLongCANTimeoutMs);
  
	  rightDrive3.config_kP(kSpinMoveControlSlot, Constants.kDriveSpinMoveKp, Constants.kLongCANTimeoutMs);
	  rightDrive3.config_kI(kSpinMoveControlSlot, Constants.kDriveSpinMoveKi, Constants.kLongCANTimeoutMs);
	  rightDrive3.config_kD(kSpinMoveControlSlot, Constants.kDriveSpinMoveKd, Constants.kLongCANTimeoutMs);
	  rightDrive3.config_kF(kSpinMoveControlSlot, Constants.kDriveSpinMoveKf, Constants.kLongCANTimeoutMs);
	  rightDrive3.config_IntegralZone(kSpinMoveControlSlot, Constants.kDriveSpinMoveIZone,
		  Constants.kLongCANTimeoutMs);
  
		  leftDrive1.config_kP(kSpinMoveControlSlot, Constants.kDriveSpinMoveKp, Constants.kLongCANTimeoutMs);
		  leftDrive1.config_kI(kSpinMoveControlSlot, Constants.kDriveSpinMoveKi, Constants.kLongCANTimeoutMs);
		  leftDrive1.config_kD(kSpinMoveControlSlot, Constants.kDriveSpinMoveKd, Constants.kLongCANTimeoutMs);
		  leftDrive1.config_kF(kSpinMoveControlSlot, Constants.kDriveSpinMoveKf, Constants.kLongCANTimeoutMs);
		  leftDrive1.config_IntegralZone(kSpinMoveControlSlot, Constants.kDriveSpinMoveIZone,
		  Constants.kLongCANTimeoutMs);
		  
	  leftDrive2.config_kP(kSpinMoveControlSlot, Constants.kDriveSpinMoveKp, Constants.kLongCANTimeoutMs);
	  leftDrive2.config_kI(kSpinMoveControlSlot, Constants.kDriveSpinMoveKi, Constants.kLongCANTimeoutMs);
	  leftDrive2.config_kD(kSpinMoveControlSlot, Constants.kDriveSpinMoveKd, Constants.kLongCANTimeoutMs);
	  leftDrive2.config_kF(kSpinMoveControlSlot, Constants.kDriveSpinMoveKf, Constants.kLongCANTimeoutMs);
	  leftDrive2.config_IntegralZone(kSpinMoveControlSlot, Constants.kDriveSpinMoveIZone,
		  Constants.kLongCANTimeoutMs);
  
	  leftDrive3.config_kP(kSpinMoveControlSlot, Constants.kDriveSpinMoveKp, Constants.kLongCANTimeoutMs);
	  leftDrive3.config_kI(kSpinMoveControlSlot, Constants.kDriveSpinMoveKi, Constants.kLongCANTimeoutMs);
	  leftDrive3.config_kD(kSpinMoveControlSlot, Constants.kDriveSpinMoveKd, Constants.kLongCANTimeoutMs);
	  leftDrive3.config_kF(kSpinMoveControlSlot, Constants.kDriveSpinMoveKf, Constants.kLongCANTimeoutMs);
	  leftDrive3.config_IntegralZone(kSpinMoveControlSlot, Constants.kDriveSpinMoveIZone,
		  Constants.kLongCANTimeoutMs);
	}
	
	public static Drive getInstance() {
		  if (instance == null) {
			  instance = new Drive();
		  }
		  return instance;
	}
	
	// RPM Set Up
	public void resetLeftDrive1Position() {
	  leftDrive1.setSelectedSensorPosition(0);
	}
  
	public double getLeftDrive1Rotations() {
	  return leftDrive1.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION;
	}
  
	public double getLeftDrive1RPM() {
	  return leftDrive1.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION * 10.0 * 60.0;
	}
  
	public void setLeftDrive1RPM(double rpm) {
	  leftDrive1.set(ControlMode.Velocity, leftDrive1RPMToNativeUnits(rpm));
	}
  
	public double leftDrive1RPMToNativeUnits(double rpm) {
	  return rpm * DRIVE_OUTPUT_TO_ENCODER_RATIO * TICKS_PER_ROTATION / 10.0 / 60.0;
	}
  
	public void resetLeftDrive2Position() {
	  leftDrive2.setSelectedSensorPosition(0);
	}
  
	public double getLeftDrive2Rotations() {
	  return leftDrive2.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION;
	}
  
	public double getLeftDrive2RPM() {
	  return leftDrive2.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION * 10.0 * 60.0;
	}
  
	public void setLeftDrive2RPM(double rpm) {
	  leftDrive2.set(ControlMode.Velocity, leftDrive2RPMToNativeUnits(rpm));
	}
  
	public double leftDrive2RPMToNativeUnits(double rpm) {
	  return rpm * DRIVE_OUTPUT_TO_ENCODER_RATIO * TICKS_PER_ROTATION / 10.0 / 60.0;
	}
  
	public void resetLeftDrive3Position() {
	  leftDrive3.setSelectedSensorPosition(0);
	}
  
	public double getLeftDrive3Rotations() {
	  return leftDrive3.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION;
	}
  
	public double getLeftDrive3RPM() {
	  return leftDrive3.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION * 10.0 * 60.0;
	}
  
	public void setLeftDrive3RPM(double rpm) {
	  leftDrive3.set(ControlMode.Velocity, leftDrive3RPMToNativeUnits(rpm));
	}
  
	public double leftDrive3RPMToNativeUnits(double rpm) {
	  return rpm * DRIVE_OUTPUT_TO_ENCODER_RATIO * TICKS_PER_ROTATION / 10.0 / 60.0;
	}
  
	public void resetRightDrive1Position() {
	  rightDrive1.setSelectedSensorPosition(0);
	}
  
	public double getRightDrive1Rotations() {
	  return rightDrive1.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION;
	}
  
	public double getRightDrive1RPM() {
	  return rightDrive1.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION * 10.0 * 60.0;
	}
  
	public void setRightDrive1RPM(double rpm) {
	  rightDrive1.set(ControlMode.Velocity, rightDrive1RPMToNativeUnits(rpm));
	}
  
	public double rightDrive1RPMToNativeUnits(double rpm) {
	  return rpm * DRIVE_OUTPUT_TO_ENCODER_RATIO * TICKS_PER_ROTATION / 10.0 / 60.0;
	}
  
	public void resetRightDrive2Position() {
	  rightDrive2.setSelectedSensorPosition(0);
	}
  
	public double getRightDrive2Rotations() {
	  return rightDrive2.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION;
	}
  
	public double getRightDrive2RPM() {
	  return rightDrive2.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION * 10.0 * 60.0;
	}
  
	public void setRightDrive2RPM(double rpm) {
	  rightDrive2.set(ControlMode.Velocity, rightDrive2RPMToNativeUnits(rpm));
	}
  
	public double rightDrive2RPMToNativeUnits(double rpm) {
	  return rpm * DRIVE_OUTPUT_TO_ENCODER_RATIO * TICKS_PER_ROTATION / 10.0 / 60.0;
	}
  
	public void resetRightDrive3Position() {
	  rightDrive3.setSelectedSensorPosition(0);
	}
  
	public double getRightDrive3Rotations() {
	  return rightDrive3.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION;
	}
  
	public double getRightDrive3RPM() {
	  return rightDrive3.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION * 10.0 * 60.0;
	}
  
	public void setRightDrive3RPM(double rpm) {
	  rightDrive3.set(ControlMode.Velocity, rightDrive3RPMToNativeUnits(rpm));
	}
  
	public double rightDrive3RPMToNativeUnits(double rpm) {
	  return rpm * DRIVE_OUTPUT_TO_ENCODER_RATIO * TICKS_PER_ROTATION / 10.0 / 60.0;
	}
	
	// Gyro Set Up
	public void calibrateGyro() {
		  gyroPigeon.enterCalibrationMode(CalibrationMode.Temperature, TalonFXEncoder.TIMEOUT_MS);
	}
	
	public void endGyroCalibration() {
		  if (isCalibrating == true) {
				isCalibrating = false;
		  }
	}
  
	public void setGyroOffset(double offsetDeg) {
		  gyroOffsetDeg = offsetDeg;
	}
	
	public synchronized Rotation2d getGyroAngle() {
		  return mAngleAdjustment.rotateBy(Rotation2d.fromDegrees(-getGyroAngleDeg()));
	  }
	
	public synchronized void setGyroAngle(Rotation2d adjustment) {
		  resetGyro();
		  mAngleAdjustment = adjustment;
	}
	
	public synchronized double getGyroAngleDeg() {
		  gyroPigeon.getYawPitchRoll(yprPigeon);
		  return -yprPigeon[0] + gyroOffsetDeg;
	}
	
	public synchronized double getGyroPitchAngle() {
		  gyroPigeon.getYawPitchRoll(yprPigeon);
		  return yprPigeon[2];
	}
	
	public short getGyroXAccel() {
		  gyroPigeon.getBiasedAccelerometer(xyzPigeon);
		  return xyzPigeon[0];
	  }
  
	  public short getGyroYAccel() {
		  gyroPigeon.getBiasedAccelerometer(xyzPigeon);
		  return xyzPigeon[1];
	  }
  
	  public short getGyroZAccel() {
		  gyroPigeon.getBiasedAccelerometer(xyzPigeon);
		  return xyzPigeon[2];
	}
	
	public boolean checkPitchAngle() {
		  double pitchAngle = Math.abs(getGyroPitchAngle());
		  if (pitchAngle > 10) {
			  return true;
		  }
		  return false;
	  }
  
	  public synchronized void resetGyro() {
		  gyroPigeon.setYaw(0, TalonFXEncoder.TIMEOUT_MS);
		  gyroPigeon.setFusedHeading(0, TalonFXEncoder.TIMEOUT_MS);
	  }
  
	public void zeroSensors() {
	  resetLeftDrive1Position();
	  resetLeftDrive2Position();
	  resetLeftDrive3Position();
	  resetRightDrive1Position();
	  resetRightDrive2Position();
	  resetRightDrive3Position();
		  resetGyro();
	}
	
	// Auto Setup
	private void setOpenLoopVoltageRamp(double timeTo12VSec) {
	  leftDrive1.configOpenloopRamp(timeTo12VSec, TalonFXEncoder.TIMEOUT_MS);
	  leftDrive2.configOpenloopRamp(timeTo12VSec, TalonFXEncoder.TIMEOUT_MS);
	  leftDrive3.configOpenloopRamp(timeTo12VSec, TalonFXEncoder.TIMEOUT_MS);
	  rightDrive1.configOpenloopRamp(timeTo12VSec, TalonFXEncoder.TIMEOUT_MS);
	  rightDrive2.configOpenloopRamp(timeTo12VSec, TalonFXEncoder.TIMEOUT_MS);
	  rightDrive3.configOpenloopRamp(timeTo12VSec, TalonFXEncoder.TIMEOUT_MS);
	}

	// Drive
	public synchronized DriveControlMode getControlMode() {
		return driveControlMode;
	}

	public synchronized void setControlMode(DriveControlMode controlMode) {
		this.driveControlMode = controlMode;
		if (controlMode == DriveControlMode.HOLD) {
			mpStraightController.setPID(mpHoldPIDParams, kPositionControlSlot);
			leftDrive1.set(ControlMode.Position, 0);
			rightDrive1.set(ControlMode.Position, 0);
		}
		setFinished(false);
	}

	public synchronized boolean isFinished() {
		return isFinished;
	}

	public synchronized void setFinished(boolean isFinished) {
		this.isFinished = isFinished;
	}

	public synchronized void setSpeed(double speed) {
		if (speed == 0) {
			setControlMode(DriveControlMode.JOYSTICK);
		} else {
			setControlMode(DriveControlMode.MANUAL);
			rightDrive1.set(ControlMode.PercentOutput, speed);
			leftDrive1.set(ControlMode.PercentOutput, speed);
		}
	}
	
	public void setRelativeTurnMP(double relativeTurnAngleDeg, double turnRateDegPerSec, MPSoftwareTurnType turnType) {
		  mpTurnController.setMPTurnTarget(getGyroAngleDeg(), relativeTurnAngleDeg + getGyroAngleDeg(), turnRateDegPerSec,
				  MP_TURN_T1, MP_TURN_T2, turnType, TRACK_WIDTH_INCHES);
		  setControlMode(DriveControlMode.MP_TURN);
	  }
  
	  public void setRelativeMaxTurnMP(double relativeTurnAngleDeg, double turnRateDegPerSec,
			  MPSoftwareTurnType turnType) {
		  mpTurnController.setMPTurnTarget(getGyroAngleDeg(), relativeTurnAngleDeg + getGyroAngleDeg(), turnRateDegPerSec,
				  MP_MAX_TURN_T1, MP_MAX_TURN_T2, turnType, TRACK_WIDTH_INCHES);
		  setControlMode(DriveControlMode.MP_TURN);
	  }
  
	  public void setAbsoluteTurnMP(double absoluteTurnAngleDeg, double turnRateDegPerSec, MPSoftwareTurnType turnType) {
		  mpTurnController.setMPTurnTarget(getGyroAngleDeg(),
				  BHRMathUtils.adjustAccumAngleToDesired(getGyroAngleDeg(), absoluteTurnAngleDeg), turnRateDegPerSec,
				  MP_TURN_T1, MP_TURN_T2, turnType, TRACK_WIDTH_INCHES);
		  setControlMode(DriveControlMode.MP_TURN);
	  }
  
	  public void setAbsoluteMaxTurnMP(double absoluteTurnAngleDeg, double turnRateDegPerSec,
			  MPSoftwareTurnType turnType) {
		  mpTurnController.setMPTurnTarget(getGyroAngleDeg(),
				  BHRMathUtils.adjustAccumAngleToDesired(getGyroAngleDeg(), absoluteTurnAngleDeg), turnRateDegPerSec,
				  MP_MAX_TURN_T1, MP_MAX_TURN_T2, turnType, TRACK_WIDTH_INCHES);
		  setControlMode(DriveControlMode.MP_TURN);
	  }
  
	  public synchronized void setGyroLock(boolean useGyroLock, boolean snapToAbsolute0or180) { // Delete
		  if (snapToAbsolute0or180) {
			  gyroLockAngleDeg = BHRMathUtils.adjustAccumAngleToClosest180(getGyroAngleDeg());
		  } else {
			  gyroLockAngleDeg = getGyroAngleDeg();
		  }
		  this.useGyroLock = useGyroLock;
	}
	
	// MP Setup
	/**
	   * Start up velocity mode. This sets the drive train in high gear as well.
	   * 
	   * @param left_inches_per_sec
	   * @param right_inches_per_sec
	   */
	  /**
	   * Check if the drive talons are configured for velocity control
	   */
	  protected static boolean usesTalonVelocityControl(DriveControlMode state) {
		  if (state == DriveControlMode.VELOCITY_SETPOINT || state == DriveControlMode.PATH_FOLLOWING
				  || state == DriveControlMode.CAMERA_TRACK || state == DriveControlMode.SPIN_MOVE) {
			  return true;
		  }
		  return false;
	}
	
	/**
	   * Check if the drive talons are configured for position control
	   */
	  protected static boolean usesTalonPositionControl(DriveControlMode state) {
		  if (state == DriveControlMode.MP_STRAIGHT || state == DriveControlMode.MP_TURN
				  || state == DriveControlMode.HOLD) {
			  return true;
		  }
		  return false;
	  }
  
	  public synchronized void setVelocitySetpoint(double left_inches_per_sec, double right_inches_per_sec) {
		  configureFalconsForSpeedControl();
		  driveControlMode = DriveControlMode.VELOCITY_SETPOINT;
		  setVelocitySetpoint(left_inches_per_sec, right_inches_per_sec);
	  }
  
	  public synchronized void setVelocityNativeUnits(double left_velocity_ticks_per_100ms,
			  double right_velocity_ticks_per_100ms) {
		  leftDrive1.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
				  mPeriodicIO.left_feedforward + Constants.kDriveVelocityKd * mPeriodicIO.left_accel / 1023.0);
		  rightDrive1.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
				  mPeriodicIO.right_feedforward + Constants.kDriveVelocityKd * mPeriodicIO.right_accel / 1023.0);
	  }
  
  
  
  
	
	
   
	
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
