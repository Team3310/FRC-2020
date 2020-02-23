package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities.Util;

public class Turret extends SubsystemBase {

    public static enum TurretControlMode {
        MOTION_MAGIC, VELOCITY, MANUAL
    };

    // Conversions
    private static final double TURRET_OUTPUT_TO_ENCODER_RATIO = 168.0 / 28.0;
    public static final double TURRET_REVOLUTIONS_TO_ENCODER_TICKS = TURRET_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    public static final double TURRET_DEGREES_TO_ENCODER_TICKS = TURRET_REVOLUTIONS_TO_ENCODER_TICKS / 360.0;

    // Motion Magic
    private static final int kTurretMotionMagicSlot = 0;
    private TurretControlMode turretControlMode = TurretControlMode.MANUAL;

    // Motor Controllers
    private final TalonFX turretMotor;

    // Sensors
    private DigitalInput maxRevTurretSensor;
    private DigitalInput minRevTurretSensor;

    // Misc
    private double homePosition = Constants.TURRET_AUTO_HOME_POSITION_DEGREES;
    private double targetPositionTicks = 0;

    // Subsystem Instance
    private final static Turret INSTANCE = new Turret();

    // Constructor
    private Turret() {
        turretMotor = new TalonFX(Constants.TURRET_MOTOR_CAN_ID);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        turretMotor.configAllSettings(configs);

        turretMotor.setInverted(TalonFXInvertType.CounterClockwise);
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.configMotionCruiseVelocity(500);
        turretMotor.configMotionAcceleration(500);
        turretMotor.configMotionSCurveStrength(4);

        maxRevTurretSensor = new DigitalInput(Constants.TURRET_MAX_REV_SENSOR_DIO_ID);
        minRevTurretSensor = new DigitalInput(Constants.TURRET_MIN_REV_SENSOR_DIO_ID);

        final StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 120;
        statorCurrentConfigs.enable = true;
        turretMotor.configStatorCurrentLimit(statorCurrentConfigs);

        turretMotor.config_kF(kTurretMotionMagicSlot, 0.4);
        turretMotor.config_kP(kTurretMotionMagicSlot, 1.0);
        turretMotor.config_kI(kTurretMotionMagicSlot, 0.0);
        turretMotor.config_kD(kTurretMotionMagicSlot, 0.0);
    }

    public static Turret getInstance() {
        return INSTANCE;
    }

    // Turret Control Mode
    private synchronized void setTurretControlMode(TurretControlMode controlMode) {
        this.turretControlMode = controlMode;
    }

    private synchronized TurretControlMode getTurretControlMode() {
        return this.turretControlMode;
    }

    // Manual Control
    public void setTurretSpeed(final double speed) {
        setTurretControlMode(TurretControlMode.MANUAL);
        turretMotor.set(ControlMode.PercentOutput, speed);
    }

    // Velocity Control
    public double getTurretRPM() {
        return turretMotor.getSelectedSensorVelocity() / TURRET_REVOLUTIONS_TO_ENCODER_TICKS  * 10.0 * 60.0;
    }

    public void setTurretRPM(final double rpm) {
        setTurretControlMode(TurretControlMode.VELOCITY);
        turretMotor.set(ControlMode.Velocity, TurretRPMToNativeUnits(rpm));
    }

    // Motion Magic
    public synchronized void setTurretMotionMagicPosition(double angle) {
        if (getTurretControlMode() != TurretControlMode.MOTION_MAGIC) {
            setTurretControlMode(TurretControlMode.MOTION_MAGIC);
        }
        turretMotor.selectProfileSlot(kTurretMotionMagicSlot, 0);
        targetPositionTicks = getTurretEncoderTicks(limitTurretAngle(angle));
        turretMotor.set(ControlMode.MotionMagic, targetPositionTicks);
    }

    public synchronized boolean hasFinishedTrajectory() {
        return turretControlMode == TurretControlMode.MOTION_MAGIC
                && Util.epsilonEquals(turretMotor.getActiveTrajectoryPosition(), targetPositionTicks, 5);
    }

    public synchronized double getTurretSetpointAngle() {
        return turretControlMode == TurretControlMode.MOTION_MAGIC
                ? targetPositionTicks / TURRET_DEGREES_TO_ENCODER_TICKS + homePosition
                : Double.NaN;
    }

    // Reset methods
    public synchronized void resetEncoders() {
        turretMotor.setSelectedSensorPosition(0);
    }

    public synchronized void resetEncoders(double homePosition) {
        turretMotor.setSelectedSensorPosition(0);
        this.homePosition = homePosition;
    }

    // Getters and Converters
    public double getTurretAngleDegrees() {
        return turretMotor.getSelectedSensorPosition() / TURRET_DEGREES_TO_ENCODER_TICKS;
    }

    public double TurretRPMToNativeUnits(final double rpm) {
        return rpm * TURRET_REVOLUTIONS_TO_ENCODER_TICKS / 10.0 / 60.0;
    }

    private int getTurretEncoderTicks(double angle) {
        double positionDegreesFromHome = angle - homePosition;
        return (int) (positionDegreesFromHome * TURRET_DEGREES_TO_ENCODER_TICKS);
    }

    public static double toTurretSafeAngleDegrees(double angle) {
        double result = angle % 360.0;
        if (result > 270) {
            result -= 360;
        } else if (result < -90) {
            result += 360;
        }
        return result;
    }

    private double limitTurretAngle(double targetAngle) {
        if (targetAngle < Constants.TURRET_MIN_ANGLE_DEGREES) {
            return Constants.TURRET_MIN_ANGLE_DEGREES;
        } else if (targetAngle > Constants.TURRET_MAX_ANGLE_DEGREES) {
            return Constants.TURRET_MAX_ANGLE_DEGREES;
        }

        return targetAngle;
    }
    public boolean getMaxTurretSensor(){
        return !maxRevTurretSensor.get();
    }
    public boolean getMinTurretSensor(){
        return !minRevTurretSensor.get();
    }

    public void periodic() {
        SmartDashboard.putNumber("Turret Angle", this.getTurretAngleDegrees());
        SmartDashboard.putNumber("Turret Velocity", turretMotor.getSelectedSensorVelocity());
        SmartDashboard.putBoolean("Turret Min Sensor", this.getMinTurretSensor());
        SmartDashboard.putBoolean("Turret Max Sensor", this.getMaxTurretSensor());
    }

}

