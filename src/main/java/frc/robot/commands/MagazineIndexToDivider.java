package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Turret;


public class MagazineIndexToDivider extends CommandBase {
    private final Magazine magazine;

    public MagazineIndexToDivider(Magazine magazine) {
        this.magazine = magazine;
        addRequirements(magazine);
    }

    @Override
    public void execute() {
        double magazineAngle = closestDividerAngle(Turret.getInstance().getTurretAngleAbsoluteDegrees(), magazine.getMagazineAngleAbsoluteDegrees());
        magazine.setMagazineMotionMagicPositionAbsolute(magazineAngle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double closestDividerAngle(double turretAngle, double magazineAngle) {
        //TODO fix THIS!!!!
        return turretAngle % 72.0;
    }
}
