package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;

public class IntakeRetractAll extends SequentialCommandGroup {

    public IntakeRetractAll(Intake intake, Magazine magazine) {

       addCommands(
//               new IntakeSetRPM(intake, Constants.INTAKE_RETRACT_RPM),
               new IntakeRetractArms(intake),
               new MagazineSetRPMRotations(magazine, Constants.MAGAZINE_INTAKE_RPM, 360),
//               new WaitCommand(1.0),
               new IntakeSetSpeed(intake, 0)
               );
    }
}