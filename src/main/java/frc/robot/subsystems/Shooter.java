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
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Shooter extends SubsystemBase
{
    private TalonSRX shooter1;

    private final double SHOOTER_OUTPUT_TO_ENCODER_RATIO = 3.0;
    private final double TICKS_PER_ROTATION = 4096.0;

    public Shooter()
    {
        shooter1 = new TalonSRX(0);

        shooter1.configFactoryDefault();
        shooter1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        shooter1.configPeakCurrentLimit(2);
        shooter1.configContinuousCurrentLimit(2);

        SupplyCurrentLimitConfiguration supplyCurrentConfigs = new SupplyCurrentLimitConfiguration();
        supplyCurrentConfigs.currentLimit = 2;
        supplyCurrentConfigs.enable = true;
        shooter1.configSupplyCurrentLimit(supplyCurrentConfigs);

        shooter1.configClosedLoopPeakOutput(0, 0.5);
        shooter1.enableCurrentLimit(true);

        shooter1.config_kF(0, 0.03);
        shooter1.config_kP(0, 0.02);
        shooter1.config_kI(0, 0.0001);
        shooter1.config_kD(0, 0.0);
    }

    public void setShooterSpeed(double speed) {
        shooter1.set(ControlMode.PercentOutput, speed);
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

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Rotations", getShooterRotations());
        SmartDashboard.putNumber("Shooter RPM", getShooterRPM());
        SmartDashboard.putNumber("Shooter Velocity Native", shooter1.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Shooter Stator Current", shooter1.getStatorCurrent());
        SmartDashboard.putNumber("Shooter Supply Current", shooter1.getSupplyCurrent());
     }
}
