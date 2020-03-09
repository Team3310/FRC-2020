package frc.robot.auto.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterAutoDoubleShotTrack extends ParallelCommandGroup {

    public ShooterAutoDoubleShotTrack(Shooter shooter, Turret turret) {
        addCommands(
                new InstantCommand(()-> Limelight.getInstance().setPipeline(Constants.LIMELIGHT_LEG_PIPELINE)),
                new LimelightSetLED(Limelight.getInstance(), Limelight.LightMode.ON),
                new ShooterSetRPM(shooter, Constants.SHOOTER_MAIN_AUTO_DOUBLE_RPM, Constants.SHOOTER_KICKER_AUTO_DOUBLE_RPM),
                new HoodSetAngle(shooter, Constants.HOOD_AUTO_DOUBLE_ANGLE_DEGREES),
                new SequentialCommandGroup(
                        new TurretSetToGyroAngle(turret, Constants.TURRET_GYRO_OFFSET_AUTO_DOUBLE_SHOT_ANGLE_DEGREES),
                        new TurretSetToTrackLimelightAngle(turret, Constants.LIMELIGHT_OFFSET_AUTO_DOUBLE_SHOT_DEGREES)
                )
        );
    }
}