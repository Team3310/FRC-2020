package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Turret;

public class ClimbLevelOne extends SequentialCommandGroup {

    public ClimbLevelOne(Intake intake, Turret turret, Magazine magazine) {
        addCommands(
                new InstantCommand(()-> intake.resetIntakeEncoder()),
                new InstantCommand(()-> intake.setClimbMotionMagicPositionAbsolute(Constants.CLIMB_LEVEL_1_INCHES)),
                new InstantCommand(()-> intake.climbLock())
        );
    }
}