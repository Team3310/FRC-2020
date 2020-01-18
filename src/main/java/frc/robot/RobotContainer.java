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
        SmartDashboard.putData("Set Shooters Speed 1.0",  new InstantCommand(()-> shooterSubsystem.setShooterSpeed(1.0)));
        SmartDashboard.putData("Set Shooters Speed 0.95",  new InstantCommand(()-> shooterSubsystem.setShooterSpeed(0.95)));
        SmartDashboard.putData("Set Shooters Speed 0.85",  new InstantCommand(()-> shooterSubsystem.setShooterSpeed(0.85)));
        SmartDashboard.putData("Set Shooters Speed 0.9",  new InstantCommand(()-> shooterSubsystem.setShooterSpeed(0.9)));
        SmartDashboard.putData("Set Shooters Speed 0.8",  new InstantCommand(()-> shooterSubsystem.setShooterSpeed(0.8)));
        SmartDashboard.putData("Set Shooters Speed 0.7",  new InstantCommand(()-> shooterSubsystem.setShooterSpeed(0.7)));
        SmartDashboard.putData("Set Shooters Speed 0.6",  new InstantCommand(()-> shooterSubsystem.setShooterSpeed(0.6)));
        SmartDashboard.putData("Set Shooters Speed 0.5",  new InstantCommand(()-> shooterSubsystem.setShooterSpeed(0.5)));
        SmartDashboard.putData("Set Shooters Speed Off",  new InstantCommand(()-> shooterSubsystem.setShooterSpeed(0.0)));
        SmartDashboard.putData("Set Shooters RPM 7500",    new InstantCommand(()-> shooterSubsystem.setShooterRPM(7500)));
        SmartDashboard.putData("Set Shooters RPM 7200",    new InstantCommand(()-> shooterSubsystem.setShooterRPM(7200)));
        SmartDashboard.putData("Set Shooters RPM 7100",    new InstantCommand(()-> shooterSubsystem.setShooterRPM(7100)));
        SmartDashboard.putData("Set Shooters RPM 7000",    new InstantCommand(()-> shooterSubsystem.setShooterRPM(7000)));
        SmartDashboard.putData("Reset Shooters Position", new InstantCommand(()-> shooterSubsystem.resetShooterPosition()));
        SmartDashboard.putData("Set Kicker Speed 1.0",  new InstantCommand(()-> shooterSubsystem.setKickerSpeed(1.0)));
        SmartDashboard.putData("Set Kicker Speed 0.95",  new InstantCommand(()-> shooterSubsystem.setKickerSpeed(0.95)));
        SmartDashboard.putData("Set Kicker Speed 0.85",  new InstantCommand(()-> shooterSubsystem.setKickerSpeed(0.85)));
        SmartDashboard.putData("Set Kicker Speed 0.9",  new InstantCommand(()-> shooterSubsystem.setKickerSpeed(0.9)));
        SmartDashboard.putData("Set Kicker Speed 0.8",  new InstantCommand(()-> shooterSubsystem.setKickerSpeed(0.8)));
        SmartDashboard.putData("Set Kicker Speed 0.7",  new InstantCommand(()-> shooterSubsystem.setKickerSpeed(0.7)));
        SmartDashboard.putData("Set Kicker Speed 0.6",  new InstantCommand(()-> shooterSubsystem.setKickerSpeed(0.6)));
        SmartDashboard.putData("Set Kicker Speed 0.5",  new InstantCommand(()-> shooterSubsystem.setKickerSpeed(0.5)));
        SmartDashboard.putData("Set Kicker Speed Off",  new InstantCommand(()-> shooterSubsystem.setKickerSpeed(0.0)));
        SmartDashboard.putData("Set Kicker RPM 7500",    new InstantCommand(()-> shooterSubsystem.setKickerRPM(7500)));
        SmartDashboard.putData("Set Kicker RPM 7200",    new InstantCommand(()-> shooterSubsystem.setKickerRPM(7200)));
        SmartDashboard.putData("Set Kicker RPM 7100",    new InstantCommand(()-> shooterSubsystem.setKickerRPM(7100)));
        SmartDashboard.putData("Set Kicker RPM 7000",    new InstantCommand(()-> shooterSubsystem.setKickerRPM(7000)));
        SmartDashboard.putData("Reset Kicker Position", new InstantCommand(()-> shooterSubsystem.resetKickerPosition()));
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
