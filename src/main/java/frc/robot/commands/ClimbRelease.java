package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Turret;

public class ClimbRelease extends SequentialCommandGroup {

    public ClimbRelease(Intake intake, Turret turret, Magazine magazine) {
        addCommands(
                new ParallelCommandGroup(
                        new InstantCommand(() -> intake.extendIntakeInnerArms()),
                        new TurretSetAngle(turret, Constants.TURRET_CLIMB_LEVEL_1_ANGLE_DEGREES),
                        new InstantCommand(() -> intake.climbPTOEngage())
                ),
                new InstantCommand(()-> intake.climbRelease())
        );
    }
}