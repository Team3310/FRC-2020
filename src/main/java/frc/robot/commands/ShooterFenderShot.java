package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

public class ShooterFenderShot extends SequentialCommandGroup {

    public ShooterFenderShot(Shooter shooter, Magazine magazine, Limelight limelight) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new FooCommand(), new BarCommand());
        addCommands(new ParallelCommandGroup(

                // Set RPMs
                new ShooterSetRPM(shooter, 2100, 2100),

                // Hood Angle
                new HoodSetAngle(shooter, 0),

                // Limelight
                new LimelightSetPipeline(limelight,1)
        ),

        // Current Limit
        new MagazineSetRPMLimit(magazine, 60, 5));
    }
}