package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Turret;


public class MagazineSetRPMLimit extends ExtraTimeoutCommand {
    private final Magazine magazine;
    private double rpm;
    private double statorCurrentLimit;

    public MagazineSetRPMLimit(Magazine magazine, double rpm, double statorCurrentLimit) {
        this.magazine = magazine;
        this.rpm = rpm;
        this.statorCurrentLimit = statorCurrentLimit;
    }

    @Override
    public void initialize() {
        magazine.setMagazineRPM(rpm);
        resetExtraOneTimer();
        startExtraOneTimeout(2.0);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        if (isExtraOneTimedOut() && magazine.getStatorCurrent() > statorCurrentLimit) {
            System.out.println("Stator current exceeded = " + magazine.getStatorCurrent() + ", timeout = " + timeSinceExtraOneInitialized());
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        magazine.setMagazineSpeed(0);
        System.out.println("Stator current limit end, interupted = " + interrupted);
    }
}
