package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;

public class IntakeRetractAll extends SequentialCommandGroup {

    public IntakeRetractAll(Intake intake, Magazine magazine) {

       addCommands(
               new IntakeSetSpeed(intake, 0),

               // Current Limit
               new MagazineSetSpeed(magazine, 0),

               // Arm State
               new IntakeRetractArms(intake)
       );
    }
}