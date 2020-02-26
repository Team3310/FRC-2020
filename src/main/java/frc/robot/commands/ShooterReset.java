package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterReset extends SequentialCommandGroup {
    public ShooterReset(Shooter shooter, Magazine magazine, Turret turret) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new FooCommand(), new BarCommand());
        addCommands(
                new ParallelCommandGroup(
                        // Turret
                        new TurretAutoZero(turret),

                        // Hood
                        new HoodSetAngle(shooter, 0),

                        // Magazine
                        new MagazineSetSpeed(magazine, 0),

                        // Shooter RPM
                        new ShooterSetSpeed(shooter, 0, 0),
                        new ShooterIntakeSetSpeed(shooter, 0)
                )
        );
    }
}