package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class ShooterShoot extends SequentialCommandGroup {

    public ShooterShoot(Shooter shooter, Magazine magazine) {
        addCommands(
                new ShooterIsReady(shooter),
                new MagazineSetRPMLimit(magazine, Constants.MAGAZINE_SHOOT_RPM, Constants.MAGAZINE_JAM_STATOR_CURRENT)
         );
    }
}