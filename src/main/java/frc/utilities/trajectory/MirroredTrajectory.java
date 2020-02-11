package frc.utilities.trajectory;

import frc.utilities.geometry.Pose2dWithCurvature;
import frc.utilities.trajectory.timing.TimedState;

public class MirroredTrajectory {
    public MirroredTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> right) {
            this.right = right;
            this.left = TrajectoryUtil.mirrorTimed(right);
    }

    public Trajectory<TimedState<Pose2dWithCurvature>> get(boolean left) {
            return left ? this.left : this.right;
    }

    public final Trajectory<TimedState<Pose2dWithCurvature>> left;
    public final Trajectory<TimedState<Pose2dWithCurvature>> right;
}