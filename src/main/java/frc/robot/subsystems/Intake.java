package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    // Conversions
    private static final double INTAKE_ROLLER_OUTPUT_TO_ENCODER_RATIO = 30.0 / 12.0;
    public static final double INTAKE_ROLLER_REVOLUTIONS_TO_ENCODER_TICKS = INTAKE_ROLLER_OUTPUT_TO_ENCODER_RATIO * Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION;

    // Motor Controllers
    private TalonFX intakeMotor;

    // Pneumatics
    private Solenoid intakeOuterArm;
    private Solenoid intakeInnerArm;

    // Misc
    private static final int kIntakeVelocitySlot = 0;

    private final static Intake INSTANCE = new Intake();

    private Intake() {
        intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_CAN_ID);

        intakeOuterArm = new Solenoid(Constants.INTAKE_OUTER_ARM_PCM_ID);
        intakeInnerArm = new Solenoid(Constants.INTAKE_INNER_ARM_PCM_ID);

        TalonFXConfiguration configs = new TalonFXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        intakeMotor.configAllSettings(configs);

        intakeMotor.setInverted(TalonFXInvertType.CounterClockwise);
        intakeMotor.setNeutralMode(NeutralMode.Brake);

        final SupplyCurrentLimitConfiguration supplyCurrentConfigs = new SupplyCurrentLimitConfiguration();
        supplyCurrentConfigs.currentLimit = 30;
        supplyCurrentConfigs.enable = true;
        intakeMotor.configSupplyCurrentLimit(supplyCurrentConfigs);

        intakeMotor.config_kF(kIntakeVelocitySlot, 0.055);
        intakeMotor.config_kP(kIntakeVelocitySlot, 0.10);
        intakeMotor.config_kI(kIntakeVelocitySlot, 0.0001);
        intakeMotor.config_kD(kIntakeVelocitySlot, 0.0);
        intakeMotor.config_IntegralZone(kIntakeVelocitySlot, (int)this.RollerRPMToNativeUnits(200));
    }

    public static Intake getInstance() {
        return INSTANCE;
    }

    public void setRollerSpeed(double speed) {
        this.intakeMotor.set(ControlMode.PercentOutput, speed);
        System.out.println("Set Intake Speed");
    }

    public void resetRollerPosition() {
        this.intakeMotor.setSelectedSensorPosition(0);
    }

    public double getRollerRotations() {
        return intakeMotor.getSelectedSensorPosition() / Constants.ENCODER_TICKS_PER_MOTOR_REVOLUTION / INTAKE_ROLLER_OUTPUT_TO_ENCODER_RATIO;
    }

    public double getRollerRPM() {
        return intakeMotor.getSelectedSensorVelocity() / INTAKE_ROLLER_REVOLUTIONS_TO_ENCODER_TICKS * 10.0 * 60.0;
    }

    public void setRollerRPM(double rpm) {
        this.intakeMotor.set(ControlMode.Velocity, this.RollerRPMToNativeUnits(rpm));
    }

    public double RollerRPMToNativeUnits(double rpm) {
        return rpm * INTAKE_ROLLER_REVOLUTIONS_TO_ENCODER_TICKS / 10.0D / 60.0D;
    }

    public void releaseIntakeArms() {
        intakeInnerArm.set(true);
        intakeOuterArm.set(true);
    }

    public void retractIntakeArms() {
        intakeInnerArm.set(false);
        intakeOuterArm.set(false);
    }


    public void periodic() {
        SmartDashboard.putNumber("Intake Roller Rotations", this.getRollerRotations());
        SmartDashboard.putNumber("Intake Roller RPM", this.getRollerRPM());
    }
}

