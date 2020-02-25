package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.controller.GameController;

public class Drive extends SubsystemBase {

    public static enum DriveControlMode {
        JOYSTICK, MP_STRAIGHT, MP_TURN, PID_TURN, HOLD, MANUAL, VELOCITY_SETPOINT, CAMERA_TRACK, PATH_FOLLOWING, OPEN_LOOP,
        CAMERA_TRACK_DRIVE, SPIN_MOVE
    };

    // Speed Control
    private static final double STEER_NON_LINEARITY = 0.5;
    private static final double MOVE_NON_LINEARITY = 1.0;

    private static final int MOVE_NON_LINEAR = 0;
    private static final int STEER_NON_LINEAR = -3;

    private static final double MOVE_SCALE = 1.0;
    private static final double STEER_SCALE = 0.85;

    private static final double MOVE_TRIM = 0.0;
    private static final double STEER_TRIM = 0.0;

    private static final double STICK_DEADBAND = 0.02;

    public static final double OPEN_LOOP_PERCENT_OUTPUT_LO = 0.6;
    public static final double OPEN_LOOP_PERCENT_OUTPUT_HI = 1.0;

    public static final double OPEN_LOOP_VOLTAGE_RAMP_HI = 0.1;
    public static final double OPEN_LOOP_VOLTAGE_RAMP_LO = 0.1;

    private double m_moveInput = 0.0;
    private double m_steerInput = 0.0;

    private double m_moveOutput = 0.0;
    private double m_steerOutput = 0.0;

    private boolean isHighGear = false;

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
    private boolean isCalibrating = false;
    private double gyroYawOffsetAngleDeg = Constants.DRIVE_COMPETITION_GYRO_HOME_ANGLE_DEGREES;

    // Differential Drive
    private DifferentialDrive m_drive;
    private GameController m_driverController;

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
        leftDriveSlave1.setInverted(TalonFXInvertType.FollowMaster);
        leftDriveSlave2.setInverted(TalonFXInvertType.FollowMaster);

        leftDriveMaster.setNeutralMode(NeutralMode.Brake);
        leftDriveSlave1.setNeutralMode(NeutralMode.Brake);
        leftDriveSlave2.setNeutralMode(NeutralMode.Brake);

        leftDriveSlave1.follow(leftDriveMaster);
        leftDriveSlave2.follow(leftDriveMaster);

        rightDriveMaster.setInverted(TalonFXInvertType.Clockwise);
        rightDriveSlave1.setInverted(TalonFXInvertType.FollowMaster);
        rightDriveSlave2.setInverted(TalonFXInvertType.FollowMaster);

        rightDriveMaster.setNeutralMode(NeutralMode.Brake);
        rightDriveSlave1.setNeutralMode(NeutralMode.Brake);
        rightDriveSlave2.setNeutralMode(NeutralMode.Brake);

        rightDriveSlave1.follow(rightDriveMaster);
        rightDriveSlave2.follow(rightDriveMaster);

        leftDriveMaster.enableVoltageCompensation(true);
        leftDriveMaster.configVoltageCompSaturation(12.0);
        leftDriveMaster.configPeakOutputForward(+1.0f);
        leftDriveMaster.configPeakOutputReverse(-1.0f);

        rightDriveMaster.enableVoltageCompensation(true);
        rightDriveMaster.configVoltageCompSaturation(12.0);
        rightDriveMaster.configPeakOutputForward(+1.0f);
        rightDriveMaster.configPeakOutputReverse(-1.0f);

        SupplyCurrentLimitConfiguration supplyCurrentConfigs = new SupplyCurrentLimitConfiguration();
        supplyCurrentConfigs.currentLimit = 100;
        supplyCurrentConfigs.enable = false;

        leftDriveMaster.configSupplyCurrentLimit(supplyCurrentConfigs);
        leftDriveSlave1.configSupplyCurrentLimit(supplyCurrentConfigs);
        leftDriveSlave2.configSupplyCurrentLimit(supplyCurrentConfigs);

        rightDriveMaster.configSupplyCurrentLimit(supplyCurrentConfigs);
        rightDriveSlave1.configSupplyCurrentLimit(supplyCurrentConfigs);
        rightDriveSlave2.configSupplyCurrentLimit(supplyCurrentConfigs);

        StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 100;
        statorCurrentConfigs.enable = false;

        leftDriveMaster.configStatorCurrentLimit(statorCurrentConfigs);
        leftDriveSlave1.configStatorCurrentLimit(statorCurrentConfigs);
        leftDriveSlave2.configStatorCurrentLimit(statorCurrentConfigs);

        rightDriveMaster.configStatorCurrentLimit(statorCurrentConfigs);
        rightDriveSlave1.configStatorCurrentLimit(statorCurrentConfigs);
        rightDriveSlave2.configStatorCurrentLimit(statorCurrentConfigs);

        // k constants
        leftDriveMaster.config_kF(0, 0.053);
        leftDriveMaster.config_kP(0, 0.50);
        leftDriveMaster.config_kI(0, 0.00001);
        leftDriveMaster.config_kD(0, 0.0);

        rightDriveMaster.config_kF(0, 0.053);
        rightDriveMaster.config_kP(0, 0.50);
        rightDriveMaster.config_kI(0, 0.00001);
        rightDriveMaster.config_kD(0, 0.0);

        m_drive = new DifferentialDrive(leftDriveMaster, rightDriveMaster);
        m_drive.setRightSideInverted(true);
        m_drive.setSafetyEnabled(false);

        gyroPigeon = new PigeonIMU(Constants.GYRO_CAN_ID);
        gyroPigeon.configFactoryDefault();
//        gyroPigeon.setStatusFramePeriod(10, 10);

        updateOpenLoopVoltageRamp();
    }

    public static Drive getInstance() {
        return INSTANCE;
    }

    // Gyro Set Up
    public void calibrateGyro() {
        gyroPigeon.enterCalibrationMode(PigeonIMU.CalibrationMode.Temperature);
    }

    public void endGyroCalibration() {
        if (isCalibrating == true) {
            isCalibrating = false;
        }
    }

    public void setGyroYawOffset(double offsetDeg) {
        gyroYawOffsetAngleDeg = offsetDeg;
    }

    public synchronized double getGyroYawAngleDeg() {
        gyroPigeon.getYawPitchRoll(yprPigeon);
        return yprPigeon[0] + gyroYawOffsetAngleDeg;
    }

    public synchronized double getGyroFusedHeadingAngleDeg() {
        return gyroPigeon.getFusedHeading() + gyroYawOffsetAngleDeg;
    }

    public synchronized double getGyroPitchAngle() {
        gyroPigeon.getYawPitchRoll(yprPigeon);
        return yprPigeon[2];
    }

    public synchronized void resetGyroYawAngle() {
        gyroPigeon.setYaw(0);
        gyroPigeon.setFusedHeading(0);
    }

    private void updateOpenLoopVoltageRamp() {
        setOpenLoopVoltageRamp(isHighGear ? OPEN_LOOP_VOLTAGE_RAMP_HI : OPEN_LOOP_VOLTAGE_RAMP_LO);
    }

    private void setOpenLoopVoltageRamp(double timeTo12VSec) {
        leftDriveMaster.configOpenloopRamp(timeTo12VSec);
        rightDriveMaster.configOpenloopRamp(timeTo12VSec);
    }

    public synchronized void driveWithJoystick() {
        if (m_drive == null) {
            return;
        }

        boolean isHighGearPrevious = isHighGear;
        isHighGear = m_driverController.getRightBumper().get();
        if (isHighGearPrevious != isHighGear) {
            updateOpenLoopVoltageRamp();
        }

        double shiftScaleFactor = OPEN_LOOP_PERCENT_OUTPUT_LO;
        if (isHighGear == true) {
            shiftScaleFactor = OPEN_LOOP_PERCENT_OUTPUT_HI;
        }

        m_moveInput = -m_driverController.getY(GenericHID.Hand.kLeft);
        m_steerInput = m_driverController.getX(GenericHID.Hand.kRight);

        m_moveOutput = adjustForSensitivity(MOVE_SCALE * shiftScaleFactor, MOVE_TRIM, m_moveInput, MOVE_NON_LINEAR, MOVE_NON_LINEARITY);
        m_steerOutput = adjustForSensitivity(STEER_SCALE, STEER_TRIM, m_steerInput, STEER_NON_LINEAR, STEER_NON_LINEARITY);

        m_drive.arcadeDrive(m_moveOutput, m_steerOutput);
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

    private boolean inDeadZone(double input) {
        boolean inDeadZone;
        if (Math.abs(input) < STICK_DEADBAND) {
            inDeadZone = true;
        } else {
            inDeadZone = false;
        }
        return inDeadZone;
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
    public void setDriverController(GameController driverController) {
        m_driverController = driverController;
    }

    public void periodic() {
        driveWithJoystick();
        SmartDashboard.putNumber("Gyro Yaw Angle", this.getGyroYawAngleDeg());
        SmartDashboard.putNumber("Gyro Heading Angle", this.getGyroFusedHeadingAngleDeg());
    }
}

