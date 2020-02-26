package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterMediumShot extends SequentialCommandGroup {

    public ShooterMediumShot(Shooter shooter, Magazine magazine, Turret turret) {
        addCommands(
                new ShooterSetReady(shooter,false),
                new ParallelDeadlineGroup(
                        new ShooterSetRPM(shooter, Constants.SHOOTER_MAIN_MEDIUM_RPM, Constants.SHOOTER_KICKER_MEDIUM_RPM),
                        new SequentialCommandGroup(
                                new TurretSetToGyroAngle(turret),
                                new TurretSetToLimelightAngle(turret, Constants.LIMELIGHT_MEDIUM_SHOT_OFFSET_DEGREES)
                        ),
                        new MagazineIndexToDivider(magazine),
                        new HoodSetAngle(shooter, 45)
                ),
                new ShooterIntakeSetRPM(shooter, Constants.SHOOTER_INTAKE_RPM),
                new ShooterSetReady(shooter,true)
         );
    }
}