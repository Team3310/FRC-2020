package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class ShooterLongShot extends SequentialCommandGroup {

    public ShooterLongShot(Shooter shooter, Magazine magazine, Turret turret) {
        // TODO: Add your sequential commands in the super() call, e.g.
        //           super(new FooCommand(), new BarCommand());
        addCommands(
                // Turret Angle
                new TurretSetAngle(turret, -171),

                // Hood Angle
                new HoodSetAngle(shooter, Constants.HOOD_LONG_ANGLE_DEGREES),

                // Set RPMs
                new ShooterSetRPM(shooter, Constants.SHOOTER_MAIN_LONG_RPM, Constants.SHOOTER_KICKER_LONG_RPM),

                // Limelight
                // new LimelightSetPipeline(limelight,Constants.LIMELIGHT_PIPELINE),

                // magazine index

                // Shooter Intake
                new ShooterIntakeSetRPM(shooter, Constants.SHOOTER_INTAKE_RPM),

                // Current Limit
                new MagazineSetRPMLimit(magazine, Constants.MAGAZINE_SHOOT_RPM, Constants.MAGAZINE_JAM_STATOR_CURRENT)
        );


    }
}