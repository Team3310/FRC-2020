/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class TurretSetAngle extends ExtraTimeoutCommand
{
    private final Turret turret;
    private double angle;

    public TurretSetAngle(Turret subsystem, double angle)
    {
        this.turret = subsystem;
        this.angle = angle;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        turret.setTurretMotionMagicPosition(angle);
        resetExtraOneTimer();
        startExtraOneTimeout(0.1);
    }

    @Override
    public boolean isFinished() {
        System.out.println("Turret angle = " + turret.getTurretAngleDegrees());
        if (isExtraOneTimedOut() && turret.hasFinishedTrajectory()) {
            System.out.println("Turret Trajectory finished");
            return true;
        }
        return false;
    }
}
