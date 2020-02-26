package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterFenderShot extends SequentialCommandGroup {

    public ShooterFenderShot(Shooter shooter, Magazine magazine, Turret turret) {

        addCommands(

                // Turret Angle
                new TurretSetAngle(turret, 0),

                // Hood Angle
                new HoodSetAngle(shooter, Constants.HOOD_MIN_ANGLE_DEGREES),

                // Set RPMs
                new ShooterSetRPM(shooter, Constants.SHOOTER_MAIN_FENDER_RPM, Constants.SHOOTER_KICKER_FENDER_RPM),

                // Limelight
  //              new LimelightSetPipeline(limelight,Constants.LIMELIGHT_PIPELINE)

                // Shooter Intake
                new ShooterIntakeSetRPM(shooter, Constants.SHOOTER_INTAKE_RPM),

                // Current Limit
                new MagazineSetRPMLimit(magazine, Constants.MAGAZINE_SHOOT_RPM, Constants.MAGAZINE_JAM_STATOR_CURRENT)
        );



    }
}