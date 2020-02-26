/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class TurretSetToLimelightAngle extends CommandBase
{
    private final Turret turret;
    private final Limelight limelight;
    private double offsetAngleDeg;

    public TurretSetToLimelightAngle(Turret subsystem, double offsetAngleDeg)
    {
        this.turret = subsystem;
        this.offsetAngleDeg = offsetAngleDeg;
        this.limelight = Limelight.getInstance();
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        limelight.setCameraMode(Limelight.CameraMode.VISION);
        limelight.setPipeline(0);
    }

    @Override
    public void execute() {
        if (Limelight.getInstance().isOnTarget()) {
            turret.setTurretMotionMagicPositionRelative(limelight.getTx() + offsetAngleDeg);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
