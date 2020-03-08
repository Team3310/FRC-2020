/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class HoodSetToTrackGyroAngle extends CommandBase
{
    private final Shooter shooter;
    private double offsetAngleDeg;

    public HoodSetToTrackGyroAngle(Shooter subsystem, double offsetAngleDeg)
    {
        this.shooter = subsystem;
        this.offsetAngleDeg = offsetAngleDeg;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        shooter.setGyroTrackMode(offsetAngleDeg);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
