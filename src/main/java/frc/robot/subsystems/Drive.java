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
    private WPI_TalonFX leftDrive1;
    private TalonFX leftDrive2;
    private TalonFX leftDrive3;

    // Right Drive
    private WPI_TalonFX rightDrive1;
    private TalonFX rightDrive2;
    private TalonFX rightDrive3;

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
        leftDrive1 = new WPI_TalonFX(Constants.DRIVETRAIN_LEFT_MOTOR_MASTER_CAN_ID);
        leftDrive2 = new TalonFX(Constants.DRIVETRAIN_LEFT_MOTOR_SLAVE_1_CAN_ID);
        leftDrive3 = new TalonFX(Constants.DRIVETRAIN_LEFT_MOTOR_SLAVE_2_CAN_ID);

        rightDrive1 = new WPI_TalonFX(Constants.DRIVETRAIN_RIGHT_MOTOR_MASTER_CAN_ID);
        rightDrive2 = new TalonFX(Constants.DRIVETRAIN_RIGHT_MOTOR_SLAVE_1_CAN_ID);
        rightDrive3 = new TalonFX(Constants.DRIVETRAIN_RIGHT_MOTOR_SLAVE_2_CAN_ID);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        leftDrive1.configAllSettings(configs);
        leftDrive2.configAllSettings(configs);
        leftDrive3.configAllSettings(configs);
        rightDrive1.configAllSettings(configs);
        rightDrive2.configAllSettings(configs);
        rightDrive3.configAllSettings(configs);

        leftDrive1.setInverted(TalonFXInvertType.CounterClockwise);
        leftDrive1.setNeutralMode(NeutralMode.Brake);
        leftDrive2.setInverted(TalonFXInvertType.CounterClockwise);
        leftDrive2.setNeutralMode(NeutralMode.Brake);
        leftDrive3.setInverted(TalonFXInvertType.CounterClockwise);
        leftDrive3.setNeutralMode(NeutralMode.Brake);

        rightDrive1.setInverted(TalonFXInvertType.Clockwise);
        rightDrive1.setNeutralMode(NeutralMode.Brake);
        rightDrive2.setInverted(TalonFXInvertType.Clockwise);
        rightDrive2.setNeutralMode(NeutralMode.Brake);
        rightDrive3.setInverted(TalonFXInvertType.Clockwise);
        rightDrive3.setNeutralMode(NeutralMode.Brake);

        SupplyCurrentLimitConfiguration supplyCurrentConfigs = new SupplyCurrentLimitConfiguration();
        supplyCurrentConfigs.currentLimit = 30;
        supplyCurrentConfigs.enable = false;

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

        m_drive = new DifferentialDrive(leftDrive1, rightDrive1);
        m_drive.setSafetyEnabled(false);
    }

    public static Drive getInstance() {
        return INSTANCE;
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

        boolean isHighGear = m_driver.getBumperPressed(GenericHID.Hand.kRight);
        double shiftScaleFactor = 0.6;
        if (isHighGear == true) {
            shiftScaleFactor = 1.0;
        }

        m_moveInput = m_driver.getY(GenericHID.Hand.kLeft);
        m_steerInput = -m_driver.getX(GenericHID.Hand.kRight);

        m_moveOutput = adjustForSensitivity(m_moveScale * shiftScaleFactor, m_moveTrim, m_moveInput, m_moveNonLinear, MOVE_NON_LINEARITY);
        m_steerOutput = adjustForSensitivity(m_steerScale * shiftScaleFactor, m_steerTrim, m_steerInput, m_steerNonLinear,
                STEER_NON_LINEARITY);

        m_drive.arcadeDrive(-m_moveOutput, -m_steerOutput);
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

