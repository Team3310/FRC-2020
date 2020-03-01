package frc.robot.auto.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterAutoMediumShotTrack extends SequentialCommandGroup {

    public ShooterAutoMediumShotTrack(Shooter shooter, Magazine magazine, Turret turret, double magazineRotations) {
        addCommands(
                new InstantCommand(()-> Limelight.getInstance().setPipeline(Constants.LIMELIGHT_MEDIUM_PIPELINE)),
                new ParallelCommandGroup(
                        new ShooterSetRPM(shooter, Constants.SHOOTER_MAIN_MEDIUM_RPM, Constants.SHOOTER_KICKER_AUTON_SHORT_RPM),
                        new TurretSetAngle(turret, Constants.TURRET_COMPETITION_HOME_POSITION_DEGREES),
                        new HoodSetAngle(shooter, Constants.HOOD_MEDIUM_ANGLE_DEGREES)
                ),
                new TurretSetToLimelightAngle(turret, Constants.LIMELIGHT_OFFSET_MEDIUM_SHOT_DEGREES),
                new ShooterIntakeSetRPM(shooter, Constants.SHOOTER_INTAKE_RPM),
                new MagazineSetRPMRotations(magazine, Constants.MAGAZINE_SHOOT_AUTO_RPM,
                        magazineRotations)
        );
    }
}