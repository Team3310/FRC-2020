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
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Magazine;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private final Shooter shooterSubsystem = new Shooter();
    private final Magazine magazineSubsystem = new Magazine();
    private final Intake intakeSubsystem =  new Intake();
    private final Climb climbSubsystem = new Climb();

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

        SmartDashboard.putData("Test Shooters",  new InstantCommand(()-> shooterSubsystem.setShooterSpeed(0.1)));
        SmartDashboard.putData("Set Shooters Speed Off",  new InstantCommand(()-> shooterSubsystem.setShooterSpeed(0.0)));
        SmartDashboard.putData("Set Shooters RPM 7000",    new InstantCommand(()-> shooterSubsystem.setShooterRPM(7000)));
        SmartDashboard.putData("Set Shooters RPM 6500",    new InstantCommand(()-> shooterSubsystem.setShooterRPM(6500)));
        SmartDashboard.putData("Set Shooters RPM 6000",    new InstantCommand(()-> shooterSubsystem.setShooterRPM(6000)));
        SmartDashboard.putData("Set Shooters RPM 5950",    new InstantCommand(()-> shooterSubsystem.setShooterRPM(5950)));
        SmartDashboard.putData("Set Shooters RPM 5900",    new InstantCommand(()-> shooterSubsystem.setShooterRPM(5900)));
        SmartDashboard.putData("Set Shooters RPM 5850",    new InstantCommand(()-> shooterSubsystem.setShooterRPM(5850)));
        SmartDashboard.putData("Set Shooters RPM 5600",    new InstantCommand(()-> shooterSubsystem.setShooterRPM(5600)));
        SmartDashboard.putData("Set Shooters RPM 5600",    new InstantCommand(()-> shooterSubsystem.setShooterRPM(5600)));
        SmartDashboard.putData("Set Shooters RPM 5000",    new InstantCommand(()-> shooterSubsystem.setShooterRPM(5000)));
        SmartDashboard.putData("Set Shooters RPM 4700",    new InstantCommand(()-> shooterSubsystem.setShooterRPM(4700)));
        SmartDashboard.putData("Set Shooters RPM 4200",    new InstantCommand(()-> shooterSubsystem.setShooterRPM(4200)));
        SmartDashboard.putData("Set Shooters RPM 2900",    new InstantCommand(()-> shooterSubsystem.setShooterRPM(2900)));
        SmartDashboard.putData("Set Shooters RPM 1900",    new InstantCommand(()-> shooterSubsystem.setShooterRPM(1900)));
        SmartDashboard.putData("Set Shooters RPM 1200",    new InstantCommand(()-> shooterSubsystem.setShooterRPM(1200)));
        SmartDashboard.putData("Reset Shooters Position", new InstantCommand(()-> shooterSubsystem.resetShooterPosition()));
        SmartDashboard.putData("Test Kicker",  new InstantCommand(()-> shooterSubsystem.setKickerSpeed(0.1)));
        SmartDashboard.putData("Set Kicker Speed 0.5",  new InstantCommand(()-> shooterSubsystem.setKickerSpeed(0.5)));
        SmartDashboard.putData("Set Kicker Speed 0.7",  new InstantCommand(()-> shooterSubsystem.setKickerSpeed(0.7)));
        SmartDashboard.putData("Set Kicker Speed 0.9",  new InstantCommand(()-> shooterSubsystem.setKickerSpeed(0.9)));
        SmartDashboard.putData("Set Kicker Speed Off",  new InstantCommand(()-> shooterSubsystem.setKickerSpeed(0.0)));
        SmartDashboard.putData("Set Kicker RPM 7000",    new InstantCommand(()-> shooterSubsystem.setKickerRPM(7000)));
        SmartDashboard.putData("Set Kicker RPM 6500",    new InstantCommand(()-> shooterSubsystem.setKickerRPM(6500)));
        SmartDashboard.putData("Set Kicker RPM 5850",    new InstantCommand(()-> shooterSubsystem.setKickerRPM(5850)));
        SmartDashboard.putData("Set Kicker RPM 6000",    new InstantCommand(()-> shooterSubsystem.setKickerRPM(6000)));
        SmartDashboard.putData("Set Kicker RPM 5950",    new InstantCommand(()-> shooterSubsystem.setKickerRPM(5950)));
        SmartDashboard.putData("Set Kicker RPM 5900",    new InstantCommand(()-> shooterSubsystem.setKickerRPM(5900)));
        SmartDashboard.putData("Set Kicker RPM 5600",    new InstantCommand(()-> shooterSubsystem.setKickerRPM(5600)));
        SmartDashboard.putData("Set Kicker RPM 5000",    new InstantCommand(()-> shooterSubsystem.setKickerRPM(5000)));
        SmartDashboard.putData("Set Kicker RPM 4700",    new InstantCommand(()-> shooterSubsystem.setKickerRPM(4700)));
        SmartDashboard.putData("Set Kicker RPM 4200",    new InstantCommand(()-> shooterSubsystem.setKickerRPM(4200)));
        SmartDashboard.putData("Set Kicker RPM 2900",    new InstantCommand(()-> shooterSubsystem.setKickerRPM(2900)));
        SmartDashboard.putData("Set Kicker RPM 1900",    new InstantCommand(()-> shooterSubsystem.setKickerRPM(1900)));
        SmartDashboard.putData("Set Kicker RPM 1200",    new InstantCommand(()-> shooterSubsystem.setKickerRPM(1200)));
        SmartDashboard.putData("Reset Kicker Position", new InstantCommand(()-> shooterSubsystem.resetKickerPosition()));
        SmartDashboard.putData("Test Intake",  new InstantCommand(()-> shooterSubsystem.setIntakeSpeed(0.1)));
        SmartDashboard.putData("Set Intake Speed 0.5",  new InstantCommand(()-> shooterSubsystem.setIntakeSpeed(0.5)));
        SmartDashboard.putData("Set Intake Speed 0.6",  new InstantCommand(()-> shooterSubsystem.setIntakeSpeed(0.6)));
        SmartDashboard.putData("Set Intake Speed 0.7",  new InstantCommand(()-> shooterSubsystem.setIntakeSpeed(0.7)));
        SmartDashboard.putData("Set Intake Speed 0.8",  new InstantCommand(()-> shooterSubsystem.setIntakeSpeed(0.8)));
        SmartDashboard.putData("Set Intake Speed 0.9",  new InstantCommand(()-> shooterSubsystem.setIntakeSpeed(0.9)));
        SmartDashboard.putData("Set Intake Speed OFF",  new InstantCommand(()-> shooterSubsystem.setIntakeSpeed(0.0)));

        SmartDashboard.putData("Set Mag Speed 1.0", new InstantCommand(()-> magazineSubsystem.setMagSpeed(1.0)));
        SmartDashboard.putData("Set Mag Speed 0.95", new InstantCommand(() -> magazineSubsystem.setMagSpeed(.95)));
        SmartDashboard.putData("Set Mag Speed 0.9", new InstantCommand(() -> magazineSubsystem.setMagSpeed(.9)));
        SmartDashboard.putData("Set Mag Speed 0.85", new InstantCommand(() -> magazineSubsystem.setMagSpeed(0.85)));
        SmartDashboard.putData("Set Mag Speed 0.8", new InstantCommand(() -> magazineSubsystem.setMagSpeed(0.8)));
        SmartDashboard.putData("Set Mag Speed 0.7", new InstantCommand(() -> magazineSubsystem.setMagSpeed(0.7)));
        SmartDashboard.putData("Set Mag Speed 0.6", new InstantCommand(() -> magazineSubsystem.setMagSpeed(0.6)));
        SmartDashboard.putData("Set Mag Speed 0.5", new InstantCommand(() -> magazineSubsystem.setMagSpeed(0.5)));
        SmartDashboard.putData("Set Mag Speed Off", new InstantCommand(() -> magazineSubsystem.setMagSpeed(0.0)));
        SmartDashboard.putData("Set Mag RPM 7500", new InstantCommand(() -> magazineSubsystem.setMagRPM(7500)));
        SmartDashboard.putData("Set Mag RPM 7200", new InstantCommand(() -> magazineSubsystem.setMagRPM(7200)));
        SmartDashboard.putData("Set Mag RPM 7100", new InstantCommand(() -> magazineSubsystem.setMagRPM(7100)));
        SmartDashboard.putData("Set Mag RPM 7000", new InstantCommand(() -> magazineSubsystem.setMagRPM(7000)));
        SmartDashboard.putData("Reset Mag Position", new InstantCommand(() -> magazineSubsystem.resetMagPosition()));
        SmartDashboard.putData("Test Magazine",  new InstantCommand(()-> magazineSubsystem.setMagSpeed(0.1)));
    }



    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An ExampleCommand will run in autonomous
        return null;
    }
}
