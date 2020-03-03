package frc.robot.auto.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.MagazineSetRPMRotations;
import frc.robot.commands.ShooterIntakeSetRPM;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterAutoShoot extends SequentialCommandGroup {

    public ShooterAutoShoot(Shooter shooter, Magazine magazine, Turret turret, double magazineRotations) {
        addCommands(
                new ShooterIntakeSetRPM(shooter, Constants.SHOOTER_INTAKE_RPM),
                new MagazineSetRPMRotations(magazine, Constants.MAGAZINE_SHOOT_AUTO_RPM,
                        magazineRotations)
        );
    }
}