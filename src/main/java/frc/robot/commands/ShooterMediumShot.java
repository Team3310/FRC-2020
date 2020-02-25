package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class ShooterMediumShot extends SequentialCommandGroup {

    public ShooterMediumShot(Shooter shooter, Magazine magazine) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new FooCommand(), new BarCommand());
        super(new ParallelCommandGroup(

                // Set RPMs
                new ShooterSetRPM(shooter, 4000, 4000),

                // Hood Angle
                new HoodSetAngle(shooter, 20)

                // Limelight
                // Add Limelight Command Here
         ),

        // Current Limit
        new MagazineSetRPMLimit(magazine, 60, 5));

        // Shooter Intake
        new ShooterIntakeSetRPM(shooter, 3500);
    }
}