package frc.robot.auto.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterAutoLegShotTrack extends SequentialCommandGroup {

    public ShooterAutoLegShotTrack(Shooter shooter, Magazine magazine, Turret turret, double magazineRotations) {
        addCommands(
                new InstantCommand(()-> Limelight.getInstance().setPipeline(Constants.LIMELIGHT_LEG_PIPELINE)),
                new LimelightSetLED(Limelight.getInstance(), Limelight.LightMode.ON),
                new ParallelCommandGroup(
                        new ShooterSetRPM(shooter, Constants.SHOOTER_MAIN_LEG_RPM, Constants.SHOOTER_KICKER_LEG_RPM),
                        new TurretSetToTrackGyroAngle(turret, Constants.TURRET_GYRO_OFFSET_LEG_SHOT_ANGLE_DEGREES),
                        new HoodSetAngle(shooter, Constants.HOOD_LEG_ANGLE_DEGREES)
                ),
                new WaitCommand(1),
                new TurretSetToLimelightAngle(turret, Constants.LIMELIGHT_OFFSET_LEG_SHOT_DEGREES)
        );
    }
}