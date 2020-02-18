package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    // Conversions
    private final double KICKER_OUTPUT_TO_ENCODER_RATIO = 48.0/48.0;
    private final double SHOOTER_OUTPUT_TO_ENCODER_RATIO = 48.0/48.0;
    private final double INTAKE_OUTPUT_TO_ENCODER_RATIO = 48.0/48.0;

    // Motor Controllers
    private TalonFX shooterMainMaster;
    private TalonFX shooterMainSlave;
    private TalonFX shooterKicker;
    private TalonFX shooterIntake;

    private final static Shooter INSTANCE = new Shooter();

    private Shooter() {
        shooterMainMaster = new TalonFX(Constants.SHOOTER_MAIN_MOTOR_MASTER_CAN_ID);
        shooterMainSlave = new TalonFX(Constants.SHOOTER_MAIN_MOTOR_SLAVE_CAN_ID);
        shooterKicker = new TalonFX(Constants.SHOOTER_KICKER_MOTOR_CAN_ID);
        shooterIntake = new TalonFX(Constants.SHOOTER_INTAKE_MOTOR_CAN_ID);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        shooterMainMaster.configAllSettings(configs);
        shooterMainSlave.configAllSettings(configs);
        shooterKicker.configAllSettings(configs);
        shooterIntake.configAllSettings(configs);

        shooterMainMaster.setInverted(TalonFXInvertType.Clockwise);
        shooterMainMaster.setNeutralMode(NeutralMode.Coast);

        shooterMainSlave.setInverted(TalonFXInvertType.CounterClockwise);
        shooterMainSlave.setNeutralMode(NeutralMode.Coast);
        shooterMainSlave.follow(shooterMainMaster);

        shooterKicker.setInverted(TalonFXInvertType.CounterClockwise);
        shooterKicker.setNeutralMode(NeutralMode.Coast);

        shooterIntake.setInverted(TalonFXInvertType.Clockwise);
        shooterIntake.setNeutralMode(NeutralMode.Coast);

        SupplyCurrentLimitConfiguration supplyCurrentConfigs = new SupplyCurrentLimitConfiguration();
        supplyCurrentConfigs.currentLimit = 30;
        supplyCurrentConfigs.enable = true;

        shooterMainMaster.configSupplyCurrentLimit(supplyCurrentConfigs);
        shooterMainSlave.configSupplyCurrentLimit(supplyCurrentConfigs);
        shooterKicker.configSupplyCurrentLimit(supplyCurrentConfigs);
        shooterIntake.configSupplyCurrentLimit(supplyCurrentConfigs);

        shooterMainMaster.config_kF(0, 0.053);
        shooterMainMaster.config_kP(0, 0.50);
        shooterMainMaster.config_kI(0, 0.00001);
        shooterMainMaster.config_kD(0, 0.0);

        shooterKicker.config_kF(0, 0.053);
        shooterKicker.config_kP(0, 0.2);
        shooterKicker.config_kI(0, 0.0000);
        shooterKicker.config_kD(0, 0.0);  // 0.6

        shooterIntake.config_kF(0, 0.053);
        shooterIntake.config_kP(0, 0.50);
        shooterIntake.config_kI(0, 0.00001);
        shooterIntake.config_kD(0, 0.0);  // 0.6
    }

    public static Shooter getInstance() {
        return INSTANCE;
    }

    public void setShooterSpeed(double speed) {
        shooterMainMaster.set(ControlMode.PercentOutput, speed);
        System.out.println("Set Shooter Speed");
    }

    public void resetShooterPosition() {
        shooterMainMaster.setSelectedSensorPosition(0);
    }

    public double getShooterRotations() {
        return shooterMainMaster.getSelectedSensorPosition() / SHOOTER_OUTPUT_TO_ENCODER_RATIO / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;
    }

    public double getShooterRPM() {
        return shooterMainMaster.getSelectedSensorVelocity() / SHOOTER_OUTPUT_TO_ENCODER_RATIO / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION * 10.0 * 60.0;
    }

    public void setShooterRPM(double rpm) {
        shooterMainMaster.set(ControlMode.Velocity, ShooterRPMToNativeUnits(rpm));
    }

    public double ShooterRPMToNativeUnits(double rpm) {
        return rpm * SHOOTER_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / 10.0 / 60.0;
    }

    public void setKickerSpeed(double speed) {
        shooterKicker.set(ControlMode.PercentOutput, speed);
    }

    public void resetKickerPosition() {
        shooterKicker.setSelectedSensorPosition(0);
    }

    public double getKickerRotations() {
        return shooterKicker.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / KICKER_OUTPUT_TO_ENCODER_RATIO;
    }

    public double getKickerRPM() {
        return shooterKicker.getSelectedSensorVelocity() / KICKER_OUTPUT_TO_ENCODER_RATIO / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION * 10.0 * 60.0;
    }

    public void setKickerRPM(double rpm) {
        shooterKicker.set(ControlMode.Velocity, IntakeRPMToNativeUnits(rpm));
    }

    public double KickerRPMToNativeUnits(double rpm) {
        return rpm * KICKER_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / 10.0 / 60.0;
    }

    public void setIntakeSpeed(double speed) {
        shooterIntake.set(ControlMode.PercentOutput, speed);
    }

    public void resetIntakePosition() {
        shooterIntake.setSelectedSensorPosition(0);
    }

    public double getIntakeRotations() {
        return shooterIntake.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / INTAKE_OUTPUT_TO_ENCODER_RATIO;
    }

    public double getIntakeRPM() {
        return shooterIntake.getSelectedSensorVelocity() / INTAKE_OUTPUT_TO_ENCODER_RATIO / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION * 10.0 * 60.0;
    }

    public void seIntakeRPM(double rpm) {
        shooterIntake.set(ControlMode.Velocity, KickerRPMToNativeUnits(rpm));
    }

    public double IntakeRPMToNativeUnits(double rpm) {
        return rpm * INTAKE_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / 10.0 / 60.0;

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooters Rotations", getShooterRotations());
        SmartDashboard.putNumber("Shooters RPM", getShooterRPM());
//        SmartDashboard.putNumber("Shooters RPM Graph", getShooterRPM());
//        SmartDashboard.putNumber("Shooters Velocity Native", shooterMainMaster.getSelectedSensorVelocity());
//        SmartDashboard.putNumber("Shooters input", shooterMainMaster.getMotorOutputPercent());
//        SmartDashboard.putNumber("Shooters Stator Current", shooterMainMaster.getStatorCurrent());
//        SmartDashboard.putNumber("Shooters Supply Current", shooterMainMaster.getSupplyCurrent());
//        SmartDashboard.putNumber("Shooters2 Supply Current", shooterMainSlave.getSupplyCurrent());
        SmartDashboard.putNumber("Kicker Rotations", getKickerRotations());
        SmartDashboard.putNumber("Kicker RPM", getKickerRPM());
//        SmartDashboard.putNumber("Kicker RPM Graph", getKickerRPM());
//        SmartDashboard.putNumber("Kicker Velocity Native", shooterKicker.getSelectedSensorVelocity());
//        SmartDashboard.putNumber("Kicker Stator Current", shooterKicker.getStatorCurrent());
//        SmartDashboard.putNumber("Kicker Supply Current", shooterKicker.getSupplyCurrent());
        SmartDashboard.putNumber("Intake Rotations", getIntakeRotations());
        SmartDashboard.putNumber("Intake RPM", getIntakeRPM());
//        SmartDashboard.putNumber("Intake RPM Graph", getIntakeRPM());
//        SmartDashboard.putNumber("Intake Velocity Native", shooterIntake.getSelectedSensorVelocity());
//        SmartDashboard.putNumber("Intake Stator Current", shooterIntake.getStatorCurrent());
//        SmartDashboard.putNumber("Intake Supply Current", shooterIntake.getSupplyCurrent());
    }
}

