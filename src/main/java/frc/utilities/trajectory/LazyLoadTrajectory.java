package frc.utilities.trajectory;

import frc.utilities.geometry.Pose2dWithCurvature;
import frc.utilities.trajectory.timing.TimedState;

public class LazyLoadTrajectory {

    public interface TrajectoryActivator {
        Trajectory<TimedState<Pose2dWithCurvature>> activateFunction();
    }

    private MirroredTrajectory trajectory;
    private TrajectoryActivator trajectoryActivator;

    public LazyLoadTrajectory(TrajectoryActivator trajectoryActivate) {
         this.trajectoryActivator = trajectoryActivate;
    }

    public MirroredTrajectory getTrajectory() {
        return trajectory;
    }

    public void activate() {
        if (trajectory == null) {
            trajectory = new MirroredTrajectory(trajectoryActivator.activateFunction());
        }
    }
}
