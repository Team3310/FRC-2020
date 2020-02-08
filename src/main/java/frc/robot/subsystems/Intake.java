package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private TalonFX roller = new TalonFX(2);

    public Intake() {
        this.roller.configFactoryDefault();
        this.roller.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        this.roller.config_kF(0, 0.0D);
        this.roller.config_kP(0, 0.0D);
        this.roller.config_kI(0, 0.0D);
        this.roller.config_kD(0, 0.0D);
    }
    public void setRollerSpeed(double speed) {
        this.roller.set(ControlMode.PercentOutput, speed);
        System.out.println("Set Intake Speed");
    }

    public void resetRollerPosition() {
        this.roller.setSelectedSensorPosition(0);
    }

    public double getRollerRotations() {
        return (double)this.roller.getSelectedSensorPosition() / 0.6666666666666666D / 2048.0D;
    }

    public double getRollerRPM() {
        return (double)this.roller.getSelectedSensorVelocity() / 0.6666666666666666D / 2048.0D * 10.0D * 60.0D;
    }

    public void setRollerRPM(double rpm) {
        this.roller.set(ControlMode.Velocity, this.RollerRPMToNativeUnits(rpm));
    }

    public double RollerRPMToNativeUnits(double rpm) {
        return rpm * 0.6666666666666666D * 2048.0D / 10.0D / 60.0D;
    }



    public void periodic() {
        SmartDashboard.putNumber("Magazine Rotations", this.getRollerRotations());
        SmartDashboard.putNumber("Magazine RPM", this.getRollerRPM());
        SmartDashboard.putNumber("Magazine RPM Graph", this.getRollerRPM());
        SmartDashboard.putNumber("Magazine Velocity Native", (double)this.roller.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Magazine input", this.roller.getMotorOutputPercent());
        SmartDashboard.putNumber("Magazine Stator Current", this.roller.getStatorCurrent());
        SmartDashboard.putNumber("Magazine Supply Current", this.roller.getSupplyCurrent());

    }

}
