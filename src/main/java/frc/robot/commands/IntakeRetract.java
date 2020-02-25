package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;

public class IntakeRetract extends SequentialCommandGroup {

    public IntakeRetract(Intake intake, Magazine magazine) {

       addCommands(
               // Set RPMs
               new IntakeSetRPM(intake, 0), new MagazineSetRPM(magazine, 0),

               // Current Limit
               new MagazineSetRPMLimit(magazine, 0, 5),

               // Arm State
               new IntakeReleaseArms(intake)
       );
    }
}