/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase
{
    private TalonFX shooter1;
    private TalonFX shooter2;
    private TalonFX kicker1;
    private TalonFX intake1;

    private final double KICKER_OUTPUT_TO_ENCODER_RATIO = 48.0/48.0;
    private final double SHOOTER_OUTPUT_TO_ENCODER_RATIO = 48.0/48.0;
    private final double INTAKE_OUTPUT_TO_ENCODER_RATIO = 48.0/48.0;
    private final double TICKS_PER_ROTATION = 2048.0;

    public Shooter()
    {
        shooter1 = new TalonFX(18);
        shooter2 = new TalonFX(13);
        kicker1 = new TalonFX(15);
        intake1 = new TalonFX(17);

        shooter1.configFactoryDefault();
        shooter2.configFactoryDefault();
        kicker1.configFactoryDefault();
        intake1.configFactoryDefault();

        shooter1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        kicker1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        intake1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        shooter2.setInverted(true);
        shooter2.follow(shooter1);

        kicker1.setInverted(true);

        SupplyCurrentLimitConfiguration supplyCurrentConfigs = new SupplyCurrentLimitConfiguration();
        supplyCurrentConfigs.currentLimit = 30;
        supplyCurrentConfigs.enable = true;
       // kicker1.configSupplyCurrentLimit(supplyCurrentConfigs);
        shooter1.configSupplyCurrentLimit(supplyCurrentConfigs);
        shooter2.configSupplyCurrentLimit(supplyCurrentConfigs);
        intake1.configSupplyCurrentLimit(supplyCurrentConfigs);

        shooter1.config_kF(0, 0.053);
        shooter1.config_kP(0, 0.50);
        shooter1.config_kI(0, 0.00001);
        shooter1.config_kD(0, 0.0);

        kicker1.config_kF(0, 0.053);
        kicker1.config_kP(0, 0.2);
        kicker1.config_kI(0, 0.0000);
        kicker1.config_kD(0, 0.0);  // 0.6

        intake1.config_kF(0, 0.053);
        intake1.config_kP(0, 0.50);
        intake1.config_kI(0, 0.00001);
        intake1.config_kD(0, 0.0);  // 0.6

        


    }

    public void setShooterSpeed(double speed) {
        shooter1.set(ControlMode.PercentOutput, speed);
        System.out.println("Set Shooter Speed");
    }

    public void resetShooterPosition() {
        shooter1.setSelectedSensorPosition(0);
    }

    public double getShooterRotations() {
        return shooter1.getSelectedSensorPosition() / SHOOTER_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION;
    }

    public double getShooterRPM() {
        return shooter1.getSelectedSensorVelocity() / SHOOTER_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION * 10.0 * 60.0;
    }

    public void setShooterRPM(double rpm) {
        shooter1.set(ControlMode.Velocity, ShooterRPMToNativeUnits(rpm));
    }

    public double ShooterRPMToNativeUnits(double rpm) {
        return rpm * SHOOTER_OUTPUT_TO_ENCODER_RATIO * TICKS_PER_ROTATION / 10.0 / 60.0;
    }

    public void setKickerSpeed(double speed) {
        kicker1.set(ControlMode.PercentOutput, speed);
    }

    public void resetKickerPosition() {
        kicker1.setSelectedSensorPosition(0);
    }

    public double getKickerRotations() {
        return kicker1.getSelectedSensorPosition() / TICKS_PER_ROTATION / KICKER_OUTPUT_TO_ENCODER_RATIO;
    }

    public double getKickerRPM() {
        return kicker1.getSelectedSensorVelocity() / KICKER_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION * 10.0 * 60.0;
    }

    public void setKickerRPM(double rpm) {
        kicker1.set(ControlMode.Velocity, IntakeRPMToNativeUnits(rpm));
    }

    public double KickerRPMToNativeUnits(double rpm) {
        return rpm * KICKER_OUTPUT_TO_ENCODER_RATIO * TICKS_PER_ROTATION / 10.0 / 60.0;
    }
   
    public void setIntakeSpeed(double speed) {
        intake1.set(ControlMode.PercentOutput, speed);
    }

    public void resetIntakePosition() {
        intake1.setSelectedSensorPosition(0);
    }

    public double getIntakeRotations() {
        return intake1.getSelectedSensorPosition() / TICKS_PER_ROTATION / INTAKE_OUTPUT_TO_ENCODER_RATIO;
    }

    public double getIntakeRPM() {
        return intake1.getSelectedSensorVelocity() / INTAKE_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION * 10.0 * 60.0;
    }

    public void seIntakeRPM(double rpm) {
       intake1.set(ControlMode.Velocity, KickerRPMToNativeUnits(rpm));
    }

    public double IntakeRPMToNativeUnits(double rpm) {
        return rpm * INTAKE_OUTPUT_TO_ENCODER_RATIO * TICKS_PER_ROTATION / 10.0 / 60.0;
 
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooters Rotations", getShooterRotations());
        SmartDashboard.putNumber("Shooters RPM", getShooterRPM());
        SmartDashboard.putNumber("Shooters RPM Graph", getShooterRPM());
        SmartDashboard.putNumber("Shooters Velocity Native", shooter1.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Shooters input", shooter1.getMotorOutputPercent());
        SmartDashboard.putNumber("Shooters Stator Current", shooter1.getStatorCurrent());
        SmartDashboard.putNumber("Shooters Supply Current", shooter1.getSupplyCurrent());
        SmartDashboard.putNumber("Shooters2 Supply Current", shooter2.getSupplyCurrent());
        SmartDashboard.putNumber("Kicker Rotations", getKickerRotations());
        SmartDashboard.putNumber("Kicker RPM", getKickerRPM());
        SmartDashboard.putNumber("Kicker RPM Graph", getKickerRPM());
        SmartDashboard.putNumber("Kicker Velocity Native", kicker1.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Kicker Stator Current", kicker1.getStatorCurrent());
        SmartDashboard.putNumber("Kicker Supply Current", kicker1.getSupplyCurrent());
        SmartDashboard.putNumber("Intake Rotations", getIntakeRotations());
        SmartDashboard.putNumber("Intake RPM", getIntakeRPM());
        SmartDashboard.putNumber("Intake RPM Graph", getIntakeRPM());
        SmartDashboard.putNumber("Intake Velocity Native", intake1.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Intake Stator Current", intake1.getStatorCurrent());
        SmartDashboard.putNumber("Intake Supply Current", intake1.getSupplyCurrent());
     }
}
