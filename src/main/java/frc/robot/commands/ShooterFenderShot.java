package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class ShooterFenderShot extends SequentialCommandGroup {

    public ShooterFenderShot(Shooter shooter, Magazine magazine) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new FooCommand(), new BarCommand());
        addCommands(

                // Set RPMs
                new ShooterSetRPM(shooter, 2100, 2100, 2000), new MagazineSetRPM(magazine, 60),

                // Current Limit
                new MagazineSetRPMLimit(magazine, 60, 5),

                // Hood Angle
                new HoodSetAngle(shooter, 0)

                // Turret
                // Add Turret Command Here
        );
    }
}