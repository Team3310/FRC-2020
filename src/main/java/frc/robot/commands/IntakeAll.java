package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;

public class IntakeAll extends SequentialCommandGroup {
    
    public IntakeAll(Intake intake, Magazine magazine) {

        addCommands(
                // Set RPMs
                new IntakeSetRPM(intake, Constants.INTAKE_COLLECT_RPM),

                // Current Limit
                new MagazineSetRPMLimit(magazine, Constants.MAGAZINE_INTAKE_RPM, Constants.MAGAZINE_JAM_STATOR_CURRENT),

                // Arm State
                new IntakeReleaseArms(intake)
        );
    }
}