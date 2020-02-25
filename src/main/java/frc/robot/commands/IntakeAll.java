package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;

public class IntakeAll extends SequentialCommandGroup {
    
    public IntakeAll(Intake intake, Magazine magazine) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new FooCommand(), new BarCommand());
        addCommands(
                // Set RPMs
                new IntakeSetRPM(intake, 2000), new MagazineSetRPM(magazine, 60),

                // Current Limit
                new MagazineSetRPMLimit(magazine, 60, 5),

                // Arm State
                new IntakeReleaseArms(intake)
        );
    }
}