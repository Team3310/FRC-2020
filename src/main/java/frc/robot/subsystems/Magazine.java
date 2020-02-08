//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Magazine extends SubsystemBase {
    private TalonFX mag = new TalonFX(1);
    private DigitalInput magSensor1;
    private final double MAG_OUTPUT_TO_ENCODER_RATIO = 0.6666666666666666D;
    private final double TICKS_PER_ROTATION = 2048.0D;

    public Magazine() {
        this.mag.configFactoryDefault();
        this.mag.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        this.mag.config_kF(0, 0.0D);
        this.mag.config_kP(0, 0.0D);
        this.mag.config_kI(0, 0.0D);
        this.mag.config_kD(0, 0.0D);
    }

    public void setMagSpeed(double speed) {
        this.mag.set(ControlMode.PercentOutput, speed);
        System.out.println("Set Magazine Speed");
    }

    public void resetMagPosition() {
        this.mag.setSelectedSensorPosition(0);
    }

    public double getMagRotations() {
        return (double)this.mag.getSelectedSensorPosition() / 0.6666666666666666D / 2048.0D;
    }

    public double getMagRPM() {
        return (double)this.mag.getSelectedSensorVelocity() / 0.6666666666666666D / 2048.0D * 10.0D * 60.0D;
    }

    public void setMagRPM(double rpm) {
        this.mag.set(ControlMode.Velocity, this.MagRPMToNativeUnits(rpm));
    }

    public double MagRPMToNativeUnits(double rpm) {
        return rpm * 0.6666666666666666D * 2048.0D / 10.0D / 60.0D;
    }

    public boolean magSensor1() {
        return this.magSensor1.get();
    }

    public void periodic() {
        SmartDashboard.putNumber("Magazine Rotations", this.getMagRotations());
        SmartDashboard.putNumber("Magazine RPM", this.getMagRPM());
        SmartDashboard.putNumber("Magazine RPM Graph", this.getMagRPM());
        SmartDashboard.putNumber("Magazine Velocity Native", (double)this.mag.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Magazine input", this.mag.getMotorOutputPercent());
        SmartDashboard.putNumber("Magazine Stator Current", this.mag.getStatorCurrent());
        SmartDashboard.putNumber("Magazine Supply Current", this.mag.getSupplyCurrent());
        SmartDashboard.putBoolean("Magazine Sensor", this.magSensor1());
    }
}
