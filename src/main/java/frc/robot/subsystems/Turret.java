package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;
import frc.utilities.util.Util;




public class Turret extends Subsystem {
    private static Turret mInstance;

    public static enum TurretControlMode {
		MOTION_MAGIC
	};

    private final TalonFX turretTalon;

    private final double TURRET_OUTPUT_TO_ENCODER_RATIO = 48.0/48.0;
    private final double TICKS_PER_ROTATION = 2048.0;
    public static final double DEGREES_TO_ENCODER_TICKS_TURRET = 0.0; // Unknown

	// Motion Magic
    private static final int kTurretMotionMagicSlot = 1;
    private TurretControlMode turretControlMode = TurretControlMode.MOTION_MAGIC;
    
    // Misc
    private double homePosition = Constants.AUTO_HOME_POSITION_DEGREES;
    private double targetPositionTicks = 0;
	


    public Turret() {
        turretTalon = new TalonFX(0);

        turretTalon.configFactoryDefault();
        turretTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        final SupplyCurrentLimitConfiguration supplyCurrentConfigs = new SupplyCurrentLimitConfiguration();
        supplyCurrentConfigs.currentLimit = 30;
        supplyCurrentConfigs.enable = true;
        turretTalon.configSupplyCurrentLimit(supplyCurrentConfigs);

        turretTalon.config_kF(0, 0.053);
        turretTalon.config_kP(0, 0.50);
        turretTalon.config_kI(0, 0.00001);
        turretTalon.config_kD(0, 0.0);
    }

    // Turret Talon Speed Control
    private synchronized void setTurretControlMode(TurretControlMode controlMode) {
		this.turretControlMode = controlMode;
	}

	private synchronized TurretControlMode getTurretControlMode() {
		return this.turretControlMode;
    }
    
    public void setTurretSpeed(final double speed) {
        turretTalon.set(ControlMode.PercentOutput, speed);
        System.out.println("Set Shooter Speed");
    }

    public double getTurretRotations() {
        return turretTalon.getSelectedSensorPosition() / TURRET_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION;
    }

    public double getTurretRPM() {
        return turretTalon.getSelectedSensorVelocity() / TURRET_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION * 10.0 * 60.0;
    }

    public void setTurretRPM(final double rpm) {
        turretTalon.set(ControlMode.Velocity, TurretRPMToNativeUnits(rpm));
    }

    public double TurretRPMToNativeUnits(final double rpm) {
        return rpm * TURRET_OUTPUT_TO_ENCODER_RATIO * TICKS_PER_ROTATION / 10.0 / 60.0;
    }

    // Motion Magic
    public synchronized void resetEncoders() {
        turretTalon.setSelectedSensorPosition(0);
	}

	public synchronized void resetEncoders(double homePosition) {
        turretTalon.setSelectedSensorPosition(0);
		this.homePosition = homePosition;
    }
    
    public synchronized void setTurretMotionMagicPosition(double angle) {
		if (getTurretControlMode() != TurretControlMode.MOTION_MAGIC) {
			setTurretControlMode(TurretControlMode.MOTION_MAGIC);
		}
		turretTalon.selectProfileSlot(kTurretMotionMagicSlot, 0);
		targetPositionTicks = getTurretEncoderTicks(angle);
		turretTalon.set(ControlMode.MotionMagic, targetPositionTicks, DemandType.ArbitraryFeedForward, angle);
    }
    
    private int getTurretEncoderTicks(double angle) {
		double positionDegreesFromHome = angle - homePosition;
		return (int) (positionDegreesFromHome * DEGREES_TO_ENCODER_TICKS_TURRET);
    }
    
    public synchronized boolean hasFinishedTrajectory() {
		return turretControlMode == TurretControlMode.MOTION_MAGIC
				&& Util.epsilonEquals(turretTalon.getActiveTrajectoryPosition(), targetPositionTicks, 5);
    }

    public synchronized double getTurretSetpointAngle() {
		return turretControlMode == TurretControlMode.MOTION_MAGIC
				? targetPositionTicks / DEGREES_TO_ENCODER_TICKS_TURRET + homePosition
				: Double.NaN;
	}
    

    @Override
    public boolean checkSystem() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub

    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub

    }
    
}