/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class HoodSetToLimelightAngle extends ExtraTimeoutCommand
{
    private final Shooter shooter;
    private final Limelight limelight;
    private double offsetAngleDeg;
    private boolean targetFound;

    public HoodSetToLimelightAngle(Shooter subsystem, double offsetAngleDeg)
    {
        this.shooter = subsystem;
        this.offsetAngleDeg = offsetAngleDeg;
        this.limelight = Limelight.getInstance();
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        limelight.setCameraMode(Limelight.CameraMode.VISION);
        limelight.setPipeline(0);
        if (limelight.isOnTarget()) {
            targetFound = true;
            System.out.println("Limelight tracking = " + -limelight.getTx());
            shooter.setHoodMotionMagicPositionRelative(-limelight.getTx() + offsetAngleDeg);
            resetExtraOneTimer();
            startExtraOneTimeout(0.1);
        }
        else {
            targetFound = false;
        }
    }

    @Override
    public boolean isFinished() {
        if (!targetFound) {
            return true;
        }
        else if (isExtraOneTimedOut() && shooter.hasFinishedTrajectory()) {
            System.out.println("Limelight hood index angle = finished");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Hood interrupted = " + interrupted);
    }
}
