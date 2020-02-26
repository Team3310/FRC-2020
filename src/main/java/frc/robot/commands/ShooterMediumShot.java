package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterMediumShot extends SequentialCommandGroup {

    public ShooterMediumShot(Shooter shooter, Magazine magazine, Turret turret) {

        addCommands(

                // Turret Angle
                new TurretSetAngle(turret, -180),

                // Hood Angle
                new HoodSetAngle(shooter, 45),

                // Set RPMs
                new ShooterSetRPM(shooter, Constants.SHOOTER_MAIN_MEDIUM_RPM, Constants.SHOOTER_KICKER_MEDIUM_RPM),

                // Limelight
  //              new LimelightSetPipeline(limelight,1),

                // Current Limit
                new MagazineSetRPMLimit(magazine, Constants.MAGAZINE_SHOOT_RPM, Constants.MAGAZINE_JAM_STATOR_CURRENT)
        );
    }
}