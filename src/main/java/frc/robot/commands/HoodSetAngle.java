package frc.robot.commands;

import frc.robot.subsystems.Shooter;


public class HoodSetAngle extends ExtraTimeoutCommand {
    private final Shooter shooter;
    private double angle;

    public HoodSetAngle(Shooter shooter, double angle) {
        this.shooter = shooter;
        this.angle = angle;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setHoodMotionMagicPositionAbsolute(angle);
        resetExtraOneTimer();
        startExtraOneTimeout(0.1);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        System.out.println("Hood angle = " + shooter.getHoodAngleAbsoluteDegrees());
        if (isExtraOneTimedOut() && shooter.hasFinishedHoodTrajectory()) {
            System.out.println("Hood Trajectory finished");
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
