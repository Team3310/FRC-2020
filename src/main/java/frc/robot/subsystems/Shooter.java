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
    private TalonSRX shooter1;
    private TalonSRX shooter2;
    private TalonFX kicker1;

    private final double SHOOTER_OUTPUT_TO_ENCODER_RATIO = 24.0/36.0;
    private final double TICKS_PER_ROTATION = 4096.0;

    public Shooter()
    {
        shooter1 = new TalonSRX(14);
        shooter2 = new TalonSRX(15);
        kicker1 = new TalonFX(16);


  //      shooter2.setInverted(true);
   //     shooter2.follow(shooter1);

        shooter1.configFactoryDefault();
        shooter2.configFactoryDefault();
 //       shooter1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

//        shooter1.configPeakCurrentLimit(2);
//        shooter1.configContinuousCurrentLimit(2);
//
//        SupplyCurrentLimitConfiguration supplyCurrentConfigs = new SupplyCurrentLimitConfiguration();
//        supplyCurrentConfigs.currentLimit = 2;
//        supplyCurrentConfigs.enable = true;
//        shooter1.configSupplyCurrentLimit(supplyCurrentConfigs);
//
//        shooter1.configClosedLoopPeakOutput(0, 0.5);
//        shooter1.enableCurrentLimit(true);

//        shooter1.config_kF(0, 0.03);
//        shooter1.config_kP(0, 0.02);
//        shooter1.config_kI(0, 0.0001);
//        shooter1.config_kD(0, 0.0);
    }

    public void setShooterSpeed(double speed) {
        shooter1.set(ControlMode.PercentOutput, -speed);
        shooter2.set(ControlMode.PercentOutput, speed);
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
        return kicker1.getSelectedSensorPosition() / SHOOTER_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION * 2.0;
    }

    public double getKickerRPM() {
        return kicker1.getSelectedSensorVelocity() / SHOOTER_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION * 10.0 * 60.0;
    }

    public void setKickerRPM(double rpm) {
        kicker1.set(ControlMode.Velocity, KickerRPMToNativeUnits(rpm));
    }

    public double KickerRPMToNativeUnits(double rpm) {
        return rpm * SHOOTER_OUTPUT_TO_ENCODER_RATIO * TICKS_PER_ROTATION / 10.0 / 60.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooters Rotations", getShooterRotations());
        SmartDashboard.putNumber("Shooters RPM", getShooterRPM());
        SmartDashboard.putNumber("Shooters Velocity Native", shooter1.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Shooters Stator Current", shooter1.getStatorCurrent());
        SmartDashboard.putNumber("Shooters Supply Current", shooter1.getSupplyCurrent());
        SmartDashboard.putNumber("Kicker Rotations", getKickerRotations());
        SmartDashboard.putNumber("Kicker RPM", getKickerRPM());
        SmartDashboard.putNumber("Kicker Velocity Native", kicker1.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Kicker Stator Current", kicker1.getStatorCurrent());
        SmartDashboard.putNumber("Kicker Supply Current", kicker1.getSupplyCurrent());
     }
}
