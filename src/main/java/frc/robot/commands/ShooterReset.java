package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterReset extends SequentialCommandGroup {

    public ShooterReset(Shooter shooter, Magazine magazine, Turret turret) {
        addCommands(
                new MagazineSetSpeed(magazine,0),
                new ShooterSetSpeed(shooter, 0, 0),
                new ShooterIntakeSetSpeed(shooter, 0),
                new ParallelCommandGroup(
                        new HoodSetAngle(shooter,0),
                        new TurretSetAngle(turret, Constants.TURRET_COMPETITION_HOME_POSITION_DEGREES)
                )
        );
    }
}