package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;


public class ShooterSetRPM extends CommandBase {
    private final Shooter shooter;
    private double shooterMainRPM;
    private double shooterKickerRPM;
    private double shooterIntakeRPM;

    public ShooterSetRPM(Shooter shooter, double shooterMainRPM, double shooterKickerRPM, double shooterIntakeRPM) {
        this.shooter = shooter;
        this.shooterMainRPM = shooterMainRPM;
        this.shooterKickerRPM = shooterKickerRPM;
        this.shooterIntakeRPM = shooterIntakeRPM;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setMainRPM(shooterMainRPM);
        shooter.setKickerRPM(shooterKickerRPM);
        shooter.setIntakeSpeed(shooterIntakeRPM);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
