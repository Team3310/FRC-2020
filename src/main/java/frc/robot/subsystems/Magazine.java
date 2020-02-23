package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Magazine extends SubsystemBase {

    // Conversions
    private static final double MAGAZINE_OUTPUT_TO_ENCODER_RATIO = 500.0 / 11.0;
    public static final double MAGAZINE_REVOLUTIONS_TO_ENCODER_TICKS = MAGAZINE_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;

    // Motor Controllers
    private TalonFX magMotor;

    // Misc
    private static final int kMagazineVelocitySlot = 0;

    private final static Magazine INSTANCE = new Magazine();

    private Magazine() {
        magMotor = new TalonFX(Constants.MAGAZINE_MOTOR_CAN_ID);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        magMotor.configAllSettings(configs);

        magMotor.setInverted(TalonFXInvertType.Clockwise);
        magMotor.setNeutralMode(NeutralMode.Brake);

        final StatorCurrentLimitConfiguration statorCurrentConfigs = new StatorCurrentLimitConfiguration();
        statorCurrentConfigs.currentLimit = 20;
        statorCurrentConfigs.triggerThresholdCurrent = 30;
        statorCurrentConfigs.triggerThresholdTime = 1.0;
        statorCurrentConfigs.enable = true;
        magMotor.configStatorCurrentLimit(statorCurrentConfigs);

        magMotor.config_kF(kMagazineVelocitySlot, 0.040);
        magMotor.config_kP(kMagazineVelocitySlot, 0.04);//0.05
        magMotor.config_kI(kMagazineVelocitySlot, 0.0);//0.0001
        magMotor.config_kD(kMagazineVelocitySlot, 0.0);
        magMotor.config_IntegralZone(kMagazineVelocitySlot, (int)this.MagazineRPMToNativeUnits(10));
    }

    public static Magazine getInstance() {
        return INSTANCE;
    }

    public void setMagazineSpeed(double speed) {
        this.magMotor.set(ControlMode.PercentOutput, speed);
        System.out.println("Set Magazine Speed");
    }

    public void resetMagazinePosition() {
        this.magMotor.setSelectedSensorPosition(0);
    }

    public double getMagazineRPM() {
        return magMotor.getSelectedSensorVelocity() / MAGAZINE_REVOLUTIONS_TO_ENCODER_TICKS * 10.0 * 60.0;
    }
    public double getMagazineRotations() {
        return magMotor.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / MAGAZINE_OUTPUT_TO_ENCODER_RATIO;
    }
    public void setMagazineRPM(double rpm) {
        this.magMotor.set(ControlMode.Velocity, this.MagazineRPMToNativeUnits(rpm));
    }

    public double MagazineRPMToNativeUnits(double rpm) {
        return rpm * MAGAZINE_REVOLUTIONS_TO_ENCODER_TICKS / 10.0D / 60.0D;
    }

    public double getStatorCurrent() {
        return magMotor.getStatorCurrent();
    }

    public void periodic() {
        SmartDashboard.putNumber("Magazine RPM", this.getMagazineRPM());
        SmartDashboard.putNumber("Magazine Roller Rotations", this.getMagazineRotations());
        SmartDashboard.putNumber("Magazine Current", magMotor.getStatorCurrent());
    }
}

