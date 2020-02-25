package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;


public class MagazineSetRPM extends CommandBase {
    private final Magazine magazine;
    private double rpm;

    public MagazineSetRPM(Magazine magazine, double rpm) {
        this.magazine = magazine;
        this.rpm = rpm;
        addRequirements(this.magazine);
    }

    @Override
    public void initialize() {
        magazine.setMagazineRPM(rpm);
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
