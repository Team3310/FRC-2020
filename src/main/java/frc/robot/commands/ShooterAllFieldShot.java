package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterAllFieldShot extends ParallelCommandGroup {
    public ShooterAllFieldShot(Shooter shooter, Magazine magazine, Turret turret, Limelight limelight) {
       addCommands(
               new LimelightSetLED(Limelight.getInstance(), Limelight.LightMode.ON),
               new InstantCommand(()->Limelight.getInstance().setPipeline(Constants.LIMELIGHT_MEDIUM_PIPELINE)),
               new ShooterSetReady(shooter,false),
               new ShooterSetRPM(shooter, Constants.SHOOTER_MAIN_ALL_FIELD_RPM, Constants.SHOOTER_KICKER_ALL_FIELD_RPM),
               new MagazineSetRPM(magazine, Constants.MAGAZINE_SHOOT_RPM),
               new ShooterSetCachedHoodAngle(shooter, Constants.HOOD_ALL_FIELD_ANGLE_DEGREES),
               new HoodSetCachedLimelightOffset(shooter, Constants.LIMELIGHT_OFFSET_ALL_FIELD_DEGREES),
               new TurretSetCachedLimelightOffset(turret, Constants.LIMELIGHT_OFFSET_ALL_FIELD_DEGREES),
               new HoodSetToTrackGyroAngle(shooter, Constants.HOOD_GYRO_OFFSET_ALL_FIELD_DEGREES),
               new SequentialCommandGroup(
                       new HoodSetToTrackLimelightAngle(shooter, Constants.LIMELIGHT_OFFSET_ALL_FIELD_DEGREES),
                       new HoodSetToLimelightAngle(shooter, Constants.HOOD_ALL_FIELD_ANGLE_DEGREES)
               ),
               new SequentialCommandGroup(
                       new TurretSetToGyroAngle(turret, Constants.TURRET_GYRO_OFFSET_ALL_FIELD_ANGLE_DEGREES),
                       new TurretSetToTrackLimelightAngle(turret, Constants.LIMELIGHT_OFFSET_ALL_FIELD_DEGREES),
                       new ShooterSetReady(shooter, true)
               ));
    }
}