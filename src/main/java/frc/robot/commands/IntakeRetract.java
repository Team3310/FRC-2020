package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;

public class IntakeRetract extends SequentialCommandGroup {

    public IntakeRetract(Intake intake, Magazine magazine) {

       addCommands(
               // Set RPMs
               new IntakeSetSpeed(intake, 0),

               // Current Limit
               new MagazineSetSpeed(magazine, 0),

               // Arm State
               new IntakeReleaseArms(intake)
       );
    }
}