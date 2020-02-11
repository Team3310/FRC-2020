package frc.utilities.trajectory;

import frc.utilities.geometry.Pose2d;
import frc.utilities.geometry.Twist2d;

public interface IPathFollower {
    public Twist2d steer(Pose2d current_pose);

    public boolean isDone();
}
