package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterMediumShot extends SequentialCommandGroup {

    public ShooterMediumShot(Shooter shooter, Magazine magazine, Turret turret) {
        addCommands(
                new ShooterSetReady(shooter,false),
                new ParallelCommandGroup(
                        new ShooterSetRPM(shooter, Constants.SHOOTER_MAIN_MEDIUM_RPM, Constants.SHOOTER_KICKER_MEDIUM_RPM),
                        new SequentialCommandGroup(
                                new TurretSetToGyroAngle(turret, Constants.TURRET_GYRO_OFFSET_MEDIUM_SHOT_ANGLE_DEGREES),
                                new TurretSetToLimelightAngle(turret, Constants.LIMELIGHT_OFFSET_MEDIUM_SHOT_DEGREES),
                                new MagazineIndexDividerToTurret(magazine, turret)
                        ),
                        new HoodSetAngle(shooter, Constants.HOOD_MEDIUM_ANGLE_DEGREES)
                ),
                new ShooterIntakeSetRPM(shooter, Constants.SHOOTER_INTAKE_RPM),
                new ShooterSetReady(shooter,true)
         );
    }
}