package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class ShooterReset extends SequentialCommandGroup {
    public ShooterReset(Shooter shooter, Magazine magazine) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new FooCommand(), new BarCommand());
        addCommands(

                // Set RPMs
                new ShooterSetRPM(shooter, 0, 0, 0), new MagazineSetRPM(magazine, 0),

                // Current Limit
                new MagazineSetRPMLimit(magazine, 0, 5),

                // Hood Angle
                new HoodSetAngle(shooter, 40)

                // Turret
                // Add Turret Command Here
        );
    }
}