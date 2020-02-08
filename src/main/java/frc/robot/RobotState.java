package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Limelight;
import frc.robot.utilities.geometry.MovingAverageTwist2d;
import frc.robot.utilities.geometry.Pose2d;
import frc.robot.utilities.geometry.Rotation2d;
import frc.robot.utilities.geometry.Translation2d;
import frc.robot.utilities.geometry.Twist2d;
import frc.robot.utilities.util.InterpolatingDouble;
import frc.robot.utilities.util.InterpolatingTreeMap;
import frc.robot.utilities.vision.GoalTracker;
import frc.robot.utilities.vision.TargetInfo;


public final class RobotState {
    private static RobotState mInstance;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }

        return mInstance;
    }

    public static boolean isDisabled() {
      return DriverStation.getInstance().isDisabled();
    }
  
    public static boolean isEnabled() {
      return DriverStation.getInstance().isEnabled();
    }
  
    public static boolean isEStopped() {
      return DriverStation.getInstance().isEStopped();
    }
  
    public static boolean isOperatorControl() {
      return DriverStation.getInstance().isOperatorControl();
    }
  
    public static boolean isAutonomous() {
      return DriverStation.getInstance().isAutonomous();
    }
  
    public static boolean isTest() {
      return DriverStation.getInstance().isTest();
    }
  
    private RobotState() {
    }

    private GoalTracker vision_target_low_ = new GoalTracker();
    private GoalTracker vision_target_high_ = new GoalTracker();

    public synchronized void resetVision() {
        vision_target_low_.reset();
        vision_target_high_.reset();
    }

    List<Translation2d> mCameraToVisionTargetPosesLow = new ArrayList<>();
    List<Translation2d> mCameraToVisionTargetPosesHigh = new ArrayList<>();

    public synchronized void addVisionUpdate(double timestamp, List<TargetInfo> observations, Limelight source) {
        mCameraToVisionTargetPosesLow.clear();
        mCameraToVisionTargetPosesHigh.clear();

        if (observations == null || observations.isEmpty()) {
            vision_target_low_.update(timestamp, new ArrayList<>());
            vision_target_high_.update(timestamp, new ArrayList<>());
            return;
        }

        for (TargetInfo target : observations) {
            mCameraToVisionTargetPosesLow.add(getCameraToVisionTargetPose(target, false, source));
            mCameraToVisionTargetPosesHigh.add(getCameraToVisionTargetPose(target, true, source));
        }

        updatePortGoalTracker(timestamp, mCameraToVisionTargetPosesLow, vision_target_low_, source);
        updatePortGoalTracker(timestamp, mCameraToVisionTargetPosesHigh, vision_target_high_, source);
    }

    private Translation2d getCameraToVisionTargetPose(TargetInfo target, boolean high, Limelight source) {
        // Compensate for camera pitch
        Translation2d xz_plane_translation = new Translation2d(target.getX(), target.getZ()).rotateBy(source.getHorizontalPlaneToLens());
        double x = xz_plane_translation.x();
        double y = target.getY();
        double z = xz_plane_translation.y();

        // find intersection with the goal
        double differential_height = source.getLensHeight() - (high ? Constants.kPortTargetHeight : Constants.kHatchTargetHeight);
        if ((z < 0.0) == (differential_height > 0.0)) {
            double scaling = differential_height / -z;
            double distance = Math.hypot(x, y) * scaling;
            Rotation2d angle = new Rotation2d(x, y, true);
            return new Translation2d(distance * angle.cos(), distance * angle.sin());
        }

        return null;
    }

    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> field_to_vehicle_;
    private InterpolatingTreeMap<InterpolatingDouble, Rotation2d> vehicle_to_turret_;
    private Twist2d vehicle_velocity_predicted_;
    private Twist2d vehicle_velocity_measured_;
    private MovingAverageTwist2d vehicle_velocity_measured_filtered_;
    private double distance_driven_;

    private void updatePortGoalTracker(double timestamp, List<Translation2d> cameraToVisionTargetPoses, GoalTracker tracker, Limelight source) {
        if (cameraToVisionTargetPoses.size() != 2 ||
                cameraToVisionTargetPoses.get(0) == null ||
                cameraToVisionTargetPoses.get(1) == null) return;
        Pose2d cameraToVisionTarget = Pose2d.fromTranslation(cameraToVisionTargetPoses.get(0).interpolate(
                cameraToVisionTargetPoses.get(1), 0.5));

        Pose2d fieldToVisionTarget = getFieldToTurret(timestamp).transformBy(source.getTurretToLens()).transformBy(cameraToVisionTarget);
        tracker.update(timestamp, List.of(new Pose2d(fieldToVisionTarget.getTranslation(), Rotation2d.identity())));
    }

  public synchronized Pose2d getFieldToTurret(double timestamp) {
      return getFieldToVehicle(timestamp).transformBy(Pose2d.fromRotation(getVehicleToTurret(timestamp)));
  }

  public synchronized Pose2d getFieldToVehicle(double timestamp) {
    return field_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
  }

  public synchronized Rotation2d getVehicleToTurret(double timestamp) {
    return vehicle_to_turret_.getInterpolated(new InterpolatingDouble(timestamp));
  }

  public synchronized void outputToSmartDashboard() {
    SmartDashboard.putString("Robot Velocity", getMeasuredVelocity().toString());
  }

  public synchronized Twist2d getMeasuredVelocity() {
    return vehicle_velocity_measured_;
  }


}