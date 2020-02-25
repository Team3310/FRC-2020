package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {

    public static enum DriveControlMode {
        JOYSTICK, MP_STRAIGHT, MP_TURN, PID_TURN, HOLD, MANUAL, VELOCITY_SETPOINT, CAMERA_TRACK, PATH_FOLLOWING, OPEN_LOOP,
        CAMERA_TRACK_DRIVE, SPIN_MOVE
    };

    // Speed Control
    private static final double DRIVE_OUTPUT_TO_ENCODER_RATIO = 48.0 / 48.0;
    private static final double TICKS_PER_ROTATION = 2048.0;

    public static final double STEER_NON_LINEARITY = 0.5;
    public static final double MOVE_NON_LINEARITY = 1.0;

    public static final double STICK_DEADBAND = 0.02;

    // Left Drive
    private WPI_TalonFX leftDriveMaster;
    private TalonFX leftDriveSlave1;
    private TalonFX leftDriveSlave2;

    // Right Drive
    private WPI_TalonFX rightDriveMaster;
    private TalonFX rightDriveSlave1;
    private TalonFX rightDriveSlave2;

    // Gyro
    private PigeonIMU gyroPigeon;
    private double[] yprPigeon = new double[3];
    private short[] xyzPigeon = new short[3];
    private double kPGyro = 0.04;
    private boolean isCalibrating = false;
    private double gyroOffsetDeg = 0;

    private DifferentialDrive m_drive;

    private int m_moveNonLinear = 0;
    private int m_steerNonLinear = -3;

    private double m_moveScale = 1.0;
    private double m_steerScale = 0.85;

    private double m_moveInput = 0.0;
    private double m_steerInput = 0.0;

    private double m_moveOutput = 0.0;
    private double m_steerOutput = 0.0;

    private double m_moveTrim = 0.0;
    private double m_steerTrim = 0.0;

    XboxController m_driver = new XboxController(Constants.DRIVER_JOYSTICK_1_USB_ID);

    // Subsystem Instance
    private final static Drive INSTANCE = new Drive();

    private Drive() {
        leftDriveMaster = new WPI_TalonFX(Constants.DRIVETRAIN_LEFT_MOTOR_MASTER_CAN_ID);
        leftDriveSlave1 = new TalonFX(Constants.DRIVETRAIN_LEFT_MOTOR_SLAVE_1_CAN_ID);
        leftDriveSlave2 = new TalonFX(Constants.DRIVETRAIN_LEFT_MOTOR_SLAVE_2_CAN_ID);

        rightDriveMaster = new WPI_TalonFX(Constants.DRIVETRAIN_RIGHT_MOTOR_MASTER_CAN_ID);
        rightDriveSlave1 = new TalonFX(Constants.DRIVETRAIN_RIGHT_MOTOR_SLAVE_1_CAN_ID);
        rightDriveSlave2 = new TalonFX(Constants.DRIVETRAIN_RIGHT_MOTOR_SLAVE_2_CAN_ID);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        leftDriveMaster.configAllSettings(configs);
        leftDriveSlave1.configAllSettings(configs);
        leftDriveSlave2.configAllSettings(configs);
        rightDriveMaster.configAllSettings(configs);
        rightDriveSlave1.configAllSettings(configs);
        rightDriveSlave2.configAllSettings(configs);

        leftDriveMaster.setInverted(TalonFXInvertType.CounterClockwise);
        leftDriveSlave1.setInverted(TalonFXInvertType.CounterClockwise);
        leftDriveSlave2.setInverted(TalonFXInvertType.CounterClockwise);
        leftDriveMaster.setNeutralMode(NeutralMode.Brake);
        leftDriveSlave1.setNeutralMode(NeutralMode.Brake);
        leftDriveSlave2.setNeutralMode(NeutralMode.Brake);
        leftDriveSlave1.follow(leftDriveMaster);
        leftDriveSlave2.follow(leftDriveMaster);

        rightDriveMaster.setInverted(TalonFXInvertType.CounterClockwise);
        rightDriveSlave1.setInverted(TalonFXInvertType.CounterClockwise);
        rightDriveSlave2.setInverted(TalonFXInvertType.CounterClockwise);
        rightDriveMaster.setNeutralMode(NeutralMode.Brake);
        rightDriveSlave1.setNeutralMode(NeutralMode.Brake);
        rightDriveSlave2.setNeutralMode(NeutralMode.Brake);
        rightDriveSlave1.follow(rightDriveMaster);
        rightDriveSlave2.follow(rightDriveMaster);

        SupplyCurrentLimitConfiguration supplyCurrentConfigs = new SupplyCurrentLimitConfiguration();
        supplyCurrentConfigs.currentLimit = 100;
        supplyCurrentConfigs.enable = false;

        leftDriveMaster.configSupplyCurrentLimit(supplyCurrentConfigs);
        leftDriveSlave1.configSupplyCurrentLimit(supplyCurrentConfigs);
        leftDriveSlave2.configSupplyCurrentLimit(supplyCurrentConfigs);

        rightDriveMaster.configSupplyCurrentLimit(supplyCurrentConfigs);
        rightDriveSlave1.configSupplyCurrentLimit(supplyCurrentConfigs);
        rightDriveSlave2.configSupplyCurrentLimit(supplyCurrentConfigs);

        // k constants
        leftDriveMaster.config_kF(0, 0.053);
        leftDriveMaster.config_kP(0, 0.50);
        leftDriveMaster.config_kI(0, 0.00001);
        leftDriveMaster.config_kD(0, 0.0);

        leftDriveSlave1.config_kF(0, 0.053);
        leftDriveSlave1.config_kP(0, 0.50);
        leftDriveSlave1.config_kI(0, 0.00001);
        leftDriveSlave1.config_kD(0, 0.0);

        leftDriveSlave2.config_kF(0, 0.053);
        leftDriveSlave2.config_kP(0, 0.50);
        leftDriveSlave2.config_kI(0, 0.00001);
        leftDriveSlave2.config_kD(0, 0.0);

        rightDriveMaster.config_kF(0, 0.053);
        rightDriveMaster.config_kP(0, 0.50);
        rightDriveMaster.config_kI(0, 0.00001);
        rightDriveMaster.config_kD(0, 0.0);

        rightDriveSlave1.config_kF(0, 0.053);
        rightDriveSlave1.config_kP(0, 0.50);
        rightDriveSlave1.config_kI(0, 0.00001);
        rightDriveSlave1.config_kD(0, 0.0);

        rightDriveSlave2.config_kF(0, 0.053);
        rightDriveSlave2.config_kP(0, 0.50);
        rightDriveSlave2.config_kI(0, 0.00001);
        rightDriveSlave2.config_kD(0, 0.0);

        m_drive = new DifferentialDrive(leftDriveMaster, rightDriveMaster);
        m_drive.setSafetyEnabled(false);
    }

    public static Drive getInstance() {
        return INSTANCE;
    }

    // RPM Set Up
    public void resetLeftDrive1Position() {
        leftDriveMaster.setSelectedSensorPosition(0);
    }

    public double getLeftDrive1Rotations() {
        return leftDriveMaster.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION;
    }

    public double getLeftDrive1RPM() {
        return leftDriveMaster.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION * 10.0 * 60.0;
    }

    public void setLeftDrive1RPM(double rpm) {
        leftDriveMaster.set(ControlMode.Velocity, leftDrive1RPMToNativeUnits(rpm));
    }

    public double leftDrive1RPMToNativeUnits(double rpm) {
        return rpm * DRIVE_OUTPUT_TO_ENCODER_RATIO * TICKS_PER_ROTATION / 10.0 / 60.0;
    }

    public void resetLeftDrive2Position() {
        leftDriveSlave1.setSelectedSensorPosition(0);
    }

    public double getLeftDrive2Rotations() {
        return leftDriveSlave1.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION;
    }

    public double getLeftDrive2RPM() {
        return leftDriveSlave1.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION * 10.0 * 60.0;
    }

    public void setLeftDrive2RPM(double rpm) {
        leftDriveSlave1.set(ControlMode.Velocity, leftDrive2RPMToNativeUnits(rpm));
    }

    public double leftDrive2RPMToNativeUnits(double rpm) {
        return rpm * DRIVE_OUTPUT_TO_ENCODER_RATIO * TICKS_PER_ROTATION / 10.0 / 60.0;
    }

    public void resetLeftDrive3Position() {
        leftDriveSlave2.setSelectedSensorPosition(0);
    }

    public double getLeftDrive3Rotations() {
        return leftDriveSlave2.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION;
    }

    public double getLeftDrive3RPM() {
        return leftDriveSlave2.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION * 10.0 * 60.0;
    }

    public void setLeftDrive3RPM(double rpm) {
        leftDriveSlave2.set(ControlMode.Velocity, leftDrive3RPMToNativeUnits(rpm));
    }

    public double leftDrive3RPMToNativeUnits(double rpm) {
        return rpm * DRIVE_OUTPUT_TO_ENCODER_RATIO * TICKS_PER_ROTATION / 10.0 / 60.0;
    }

    public void resetRightDrive1Position() {
        rightDriveMaster.setSelectedSensorPosition(0);
    }

    public double getRightDrive1Rotations() {
        return rightDriveMaster.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION;
    }

    public double getRightDrive1RPM() {
        return rightDriveMaster.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION * 10.0 * 60.0;
    }

    public void setRightDrive1RPM(double rpm) {
        rightDriveMaster.set(ControlMode.Velocity, rightDrive1RPMToNativeUnits(rpm));
    }

    public double rightDrive1RPMToNativeUnits(double rpm) {
        return rpm * DRIVE_OUTPUT_TO_ENCODER_RATIO * TICKS_PER_ROTATION / 10.0 / 60.0;
    }

    public void resetRightDrive2Position() {
        rightDriveSlave1.setSelectedSensorPosition(0);
    }

    public double getRightDrive2Rotations() {
        return rightDriveSlave1.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION;
    }

    public double getRightDrive2RPM() {
        return rightDriveSlave1.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION * 10.0 * 60.0;
    }

    public void setRightDrive2RPM(double rpm) {
        rightDriveSlave1.set(ControlMode.Velocity, rightDrive2RPMToNativeUnits(rpm));
    }

    public double rightDrive2RPMToNativeUnits(double rpm) {
        return rpm * DRIVE_OUTPUT_TO_ENCODER_RATIO * TICKS_PER_ROTATION / 10.0 / 60.0;
    }

    public void resetRightDrive3Position() {
        rightDriveSlave2.setSelectedSensorPosition(0);
    }

    public double getRightDrive3Rotations() {
        return rightDriveSlave2.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION;
    }

    public double getRightDrive3RPM() {
        return rightDriveSlave2.getSelectedSensorPosition() / DRIVE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION * 10.0 * 60.0;
    }

    public void setRightDrive3RPM(double rpm) {
        rightDriveSlave2.set(ControlMode.Velocity, rightDrive3RPMToNativeUnits(rpm));
    }

    public double rightDrive3RPMToNativeUnits(double rpm) {
        return rpm * DRIVE_OUTPUT_TO_ENCODER_RATIO * TICKS_PER_ROTATION / 10.0 / 60.0;
    }

    // Gyro Set Up
    public void calibrateGyro() {
        gyroPigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.Temperature, 10);
    }

    public void endGyroCalibration() {
        if (isCalibrating == true) {
            isCalibrating = false;
        }
    }

    public void setGyroOffset(double offsetDeg) {
        gyroOffsetDeg = offsetDeg;
    }

    public synchronized double getGyroAngleDeg() {
        gyroPigeon.getYawPitchRoll(yprPigeon);
        return -yprPigeon[0] + gyroOffsetDeg;
    }

    public synchronized double getGyroPitchAngle() {
        gyroPigeon.getYawPitchRoll(yprPigeon);
        return yprPigeon[2];
    }

    public synchronized void resetGyro() {
        gyroPigeon.setYaw(0, 10);
        gyroPigeon.setFusedHeading(0, 10);
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

    public synchronized void driveWithJoystick() {
        if (m_drive == null)
            return;

        boolean isHighGear = m_driver.getBumper(GenericHID.Hand.kRight);
        double shiftScaleFactor = 0.6;
        if (isHighGear == true) {
            shiftScaleFactor = 1.0;
        }

        m_moveInput = -m_driver.getY(GenericHID.Hand.kLeft);
        m_steerInput = m_driver.getX(GenericHID.Hand.kRight);

  //      m_moveOutput = adjustForSensitivity(m_moveScale * shiftScaleFactor, m_moveTrim, m_moveInput, m_moveNonLinear, MOVE_NON_LINEARITY);
  //      m_steerOutput = adjustForSensitivity(m_steerScale * shiftScaleFactor, m_steerTrim, m_steerInput, m_steerNonLinear,
  //              STEER_NON_LINEARITY);

        m_moveOutput = m_moveInput * shiftScaleFactor;
        m_steerOutput = m_steerInput;

        m_drive.arcadeDrive(m_moveOutput, m_steerOutput);
        System.out.println("Drive move = " + m_moveOutput + ", Steer = " + m_steerOutput + ", Highgear = " + isHighGear);
    }

    private boolean inDeadZone(double input) {
        boolean inDeadZone;
        if (Math.abs(input) < STICK_DEADBAND) {
            inDeadZone = true;
        } else {
            inDeadZone = false;
        }
        return inDeadZone;
    }

    public double adjustForSensitivity(double scale, double trim, double steer, int nonLinearFactor,
                                       double wheelNonLinearity) {
        if (inDeadZone(steer))
            return 0;

        steer += trim;
        steer *= scale;
        steer = limitValue(steer);

        int iterations = Math.abs(nonLinearFactor);
        for (int i = 0; i < iterations; i++) {
            if (nonLinearFactor > 0) {
                steer = nonlinearStickCalcPositive(steer, wheelNonLinearity);
            } else {
                steer = nonlinearStickCalcNegative(steer, wheelNonLinearity);
            }
        }
        return steer;
    }

    private double limitValue(double value) {
        if (value > 1.0) {
            value = 1.0;
        } else if (value < -1.0) {
            value = -1.0;
        }
        return value;
    }

    private double nonlinearStickCalcPositive(double steer, double steerNonLinearity) {
        return Math.sin(Math.PI / 2.0 * steerNonLinearity * steer) / Math.sin(Math.PI / 2.0 * steerNonLinearity);
    }

    private double nonlinearStickCalcNegative(double steer, double steerNonLinearity) {
        return Math.asin(steerNonLinearity * steer) / Math.asin(steerNonLinearity);
    }

    public void periodic() {
        driveWithJoystick();
    }
}

