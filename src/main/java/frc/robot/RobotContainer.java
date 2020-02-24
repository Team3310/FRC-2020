/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.MagazineSetRPMLimit;
import frc.robot.commands.TurretAutoZero;
import frc.robot.commands.TurretSetAngle;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
    private final AirCompressor compressor = AirCompressor.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Magazine magazine = Magazine.getInstance();
    private final Turret turret = Turret.getInstance();
    private final Shooter shooter = Shooter.getInstance();

    private final TurretSetAngle autonomousCommand = new TurretSetAngle(turret, 0);

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton JoystickButton}.
     */
    private void configureButtonBindings()
    {
        SmartDashboard.putData("Intake Set Speed", new InstantCommand(()-> intake.setRollerSpeed(0.2)));
        SmartDashboard.putData("Intake Set Speed OFF", new InstantCommand(()-> intake.setRollerSpeed(0.0)));
        SmartDashboard.putData("Intake Set RPM", new InstantCommand(()-> intake.setRollerRPM(2000.0)));

        SmartDashboard.putData("Mag Set Speed", new InstantCommand(()-> magazine.setMagazineSpeed(0.2)));
        SmartDashboard.putData("Mag Set Speed OFF", new InstantCommand(()-> magazine.setMagazineSpeed(0.0)));
        SmartDashboard.putData("Mag Set RPM", new InstantCommand(()-> magazine.setMagazineRPM(60.0)));
        SmartDashboard.putData("Mag Set RPM Limit", new MagazineSetRPMLimit(magazine, 60, 5));

        SmartDashboard.putData("Shooter Main Set Speed", new InstantCommand(()-> shooter.setMainSpeed(0.2)));
        SmartDashboard.putData("Shooter Main Set OFF", new InstantCommand(()-> shooter.setMainSpeed(0.0)));
        SmartDashboard.putData("Shooter Main Set RPM Fender", new InstantCommand(()-> shooter.setMainRPM(2100)));
        SmartDashboard.putData("Shooter Main Set RPM Auton", new InstantCommand(()-> shooter.setMainRPM(3500)));
        SmartDashboard.putData("Shooter Main Set RPM Long", new InstantCommand(()-> shooter.setMainRPM(4700)));

        SmartDashboard.putData("Shooter Kicker Set Speed", new InstantCommand(()-> shooter.setKickerSpeed(0.2)));
        SmartDashboard.putData("Shooter Kicker Set OFF", new InstantCommand(()-> shooter.setKickerSpeed(0.0)));
        SmartDashboard.putData("Shooter Kicker Set RPM Fender", new InstantCommand(()-> shooter.setKickerRPM(2100)));
        SmartDashboard.putData("Shooter Kicker Set RPM Auton", new InstantCommand(()-> shooter.setKickerRPM(3500)));
        SmartDashboard.putData("Shooter Kicker Set RPM Long", new InstantCommand(()-> shooter.setKickerRPM(4700)));

        SmartDashboard.putData("Shooter Intake Set Speed", new InstantCommand(()-> shooter.setIntakeSpeed(0.2)));
        SmartDashboard.putData("Shooter Intake Set OFF", new InstantCommand(()-> shooter.setIntakeSpeed(0.0)));
        SmartDashboard.putData("Shooter Intake Set RPM Fender", new InstantCommand(()-> shooter.seIntakeRPM(2000)));
        SmartDashboard.putData("Shooter Intake Set RPM Auto", new InstantCommand(()-> shooter.seIntakeRPM(3000)));
        SmartDashboard.putData("Shooter Intake Set RPM Long", new InstantCommand(()-> shooter.seIntakeRPM(4000)));

        SmartDashboard.putData("Turret Set Speed", new InstantCommand(()-> turret.setTurretSpeed(0.2)));
        SmartDashboard.putData("Turret Set OFF", new InstantCommand(()-> turret.setTurretSpeed(0.0)));
        SmartDashboard.putData("Turret Reset", new InstantCommand(()-> turret.resetEncoders()));
        SmartDashboard.putData("Turret MM", new InstantCommand(()-> turret.setTurretMotionMagicPositionAbsolute(-135)));
        SmartDashboard.putData("Turret Position", new InstantCommand(()-> turret.setTurretPositionRelative(5)));
        SmartDashboard.putData("Turret Position Neg", new InstantCommand(()-> turret.setTurretPositionRelative(-5)));
        SmartDashboard.putData("Turret Auto Zero", new TurretAutoZero(turret));

        SmartDashboard.putData("Hood Reset", new InstantCommand(()-> shooter.resetHoodPosition()));
        SmartDashboard.putData("Hood MM", new InstantCommand(()-> shooter.setHoodMotionMagicPositionAbsolute(15)));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An ExampleCommand will run in autonomous
        return autonomousCommand;
    }
}
