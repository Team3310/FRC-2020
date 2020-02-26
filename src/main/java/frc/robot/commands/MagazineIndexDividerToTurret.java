package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.Util;


public class MagazineIndexDividerToTurret extends CommandBase {
    private final Magazine magazine;
    public static final double dividerDeltaAngle = 72.0;

    public MagazineIndexDividerToTurret(Magazine magazine) {
        this.magazine = magazine;
        addRequirements(magazine);
    }

    @Override
    public void execute() {
        double magazineAngle = closestDividerDeltaAngle(Turret.getInstance().getTurretAngleAbsoluteDegrees(), magazine.getMagazineAngleAbsoluteDegrees());
        magazine.setMagazineMotionMagicPositionAbsolute(magazineAngle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double closestDividerDeltaAngle(double turretAngle, double magazineAngle) {
        double normalizedDividerAngle = Util.normalizeAngle90ToMinus270(magazineAngle);
        double currentDividerAngle = normalizedDividerAngle;
        double closestDividerDeltaAngle = turretAngle - currentDividerAngle;
        for (int i = 1; i < 5; i++) {
            currentDividerAngle = Util.normalizeAngle90ToMinus270(currentDividerAngle + dividerDeltaAngle);
            double currentDividerDeltaAngle = turretAngle - currentDividerAngle;
            if (Math.abs(currentDividerDeltaAngle) < Math.abs(closestDividerDeltaAngle)) {
                closestDividerDeltaAngle = currentDividerDeltaAngle;
            }
        }

        return normalizedDividerAngle + closestDividerDeltaAngle;
    }
}
