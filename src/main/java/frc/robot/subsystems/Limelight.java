package frc.robot.subsystems;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Util;


import java.util.*;

public class Limelight extends SubsystemBase {
    public final static int kDefaultPipeline = 0;
    public final static int kSortTopPipeline = 1;

    public static class LimelightConstants {
        public String kName = "";
        public String kTableName = "";
        public double kHeight = 0.0;
        public Pose2d kTurretToLens = new Pose2d();
        public Rotation2d kHorizontalPlaneToLens = new Rotation2d();
        public static final double kHorizontalFOV = 59.6; // degrees
        public static final double kVerticalFOV = 49.7; // degrees
        public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
        public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
        public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds
    }

    private NetworkTable mNetworkTable;


    public Limelight(LimelightConstants constants) {
        mConstants = constants;
        mNetworkTable = NetworkTableInstance.getDefault().getTable(constants.kTableName);
    }

    public static class PeriodIO {

        // INPUTS
        public double latency;
        public int givenLedMode;
        public int givenPipeline;
        public double xOffset;
        public double yOffset;
        public double area;

        // OUTPUTS
        public int ledMode = 1; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int pipeline = 0; // 0 - 9
        public int stream = 2; // sets stream layout if another webcam is attached
        public int snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz
    }

    private LimelightConstants mConstants = null;
    private PeriodIO mPeriodicIO = new PeriodIO();
    private boolean mOutputsHaveChanged = true;
    private double[] mZeroArray = new double[]{0, 0, 0, 0, 0, 0, 0, 0};
    private List<TargetInfo> mTargets = new ArrayList<>();
    private boolean mSeesTarget = false;

    public Pose2d getTurretToLens() {
        return mConstants.kTurretToLens;
    }

    public double getLensHeight() {
        return mConstants.kHeight;
    }

    public Rotation2d getHorizontalPlaneToLens() {
        return mConstants.kHorizontalPlaneToLens;
    }

    // Target Info
    public class TargetInfo {
        protected double x = 1.0;
        protected double y;
        protected double z;
        protected double skew;


        public TargetInfo(double y, double z) {
            this.y = y;
            this.z = z;
        }

        public void setSkew(double skew) {
            this.skew = skew;
        }

        public double getX() {
            return x;
        }

        public double getY() {
            return y;
        }

        public double getZ() {
            return z;
        }

        public double getSkew() {
            return skew;
        }
    }

    public Translation2d translation_;
    public Rotation2d rotation_;




    // Periodic Inputs
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0 + mConstants.kImageCaptureLatency;
        mPeriodicIO.givenLedMode = (int) mNetworkTable.getEntry("ledMode").getDouble(1.0);
        mPeriodicIO.givenPipeline = (int) mNetworkTable.getEntry("pipeline").getDouble(0);
        mPeriodicIO.xOffset = mNetworkTable.getEntry("tx").getDouble(0.0);
        mPeriodicIO.yOffset = mNetworkTable.getEntry("ty").getDouble(0.0);
        mPeriodicIO.area = mNetworkTable.getEntry("ta").getDouble(0.0);
        mSeesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
    }

    // Periodic Outputs
    public synchronized void writePeriodicOutputs() {
        if (mPeriodicIO.givenLedMode != mPeriodicIO.ledMode ||
                mPeriodicIO.givenPipeline != mPeriodicIO.pipeline) {
            System.out.println("Table has changed from expected, retrigger!!");
            mOutputsHaveChanged = true;
        }
        if (mOutputsHaveChanged) {
            mNetworkTable.getEntry("ledMode").setNumber(mPeriodicIO.ledMode);
            mNetworkTable.getEntry("camMode").setNumber(mPeriodicIO.camMode);
            mNetworkTable.getEntry("pipeline").setNumber(mPeriodicIO.pipeline);
            mNetworkTable.getEntry("stream").setNumber(mPeriodicIO.stream);
            mNetworkTable.getEntry("snapshot").setNumber(mPeriodicIO.snapshot);

            mOutputsHaveChanged = false;
        }
    }

    public void stop() {}

    public boolean checkSystem() {
        return true;
    }

    public synchronized void outputTelemetry() {
        SmartDashboard.putBoolean(mConstants.kName + ": Has Target", mSeesTarget);
        SmartDashboard.putNumber(mConstants.kName + ": Pipeline Latency (ms)", mPeriodicIO.latency);
    }

    public enum LedMode {
        PIPELINE, OFF, BLINK, ON
    }

    public synchronized void setLed(LedMode mode) {
        if (mode.ordinal() != mPeriodicIO.ledMode) {
            mPeriodicIO.ledMode = mode.ordinal();
            mOutputsHaveChanged = true;
        }
    }

    public synchronized void setPipeline(int mode) {
        if (mode != mPeriodicIO.pipeline) {
            mPeriodicIO.pipeline = mode;

            System.out.println(mPeriodicIO.pipeline + ", " + mode);
            mOutputsHaveChanged = true;
        }
    }

    public synchronized void triggerOutputs() {
        mOutputsHaveChanged = true;
    }

    public synchronized int getPipeline() {

        return mPeriodicIO.pipeline;
    }

    public synchronized boolean seesTarget() {

        return mSeesTarget;
    }

    public synchronized List<TargetInfo> getTarget() {
        List<TargetInfo> targets = getRawTargetInfos();
        if (seesTarget() && targets != null) {
            return targets;
        }

        return null;
    }

    private synchronized List<TargetInfo> getRawTargetInfos() {
        List<double[]> corners = getTopCorners();
        if (corners == null) {
            return null;
        }

        double slope = 1.0;
        if (Math.abs(corners.get(1)[0] - corners.get(0)[0]) > Util.kEpsilon) {
            slope = (corners.get(1)[1] - corners.get(0)[1]) /
                    (corners.get(1)[0] - corners.get(0)[0]);
        }

        mTargets.clear();
        for (int i = 0; i < 2; ++i) {
            // Average of y and z;
            double y_pixels = corners.get(i)[0];
            double z_pixels = corners.get(i)[1];

            // Redefine to robot frame of reference.
            double nY = -((y_pixels - 160.0) / 160.0);
            double nZ = -((z_pixels - 120.0) / 120.0);

            double y = mConstants.kVPW / 2 * nY;
            double z = mConstants.kVPH / 2 * nZ;

            TargetInfo target = new TargetInfo(y, z);
            target.setSkew(slope);
            mTargets.add(target);
        }

        return mTargets;
    }

    private List<double[]> getTopCorners() {
        double[] xCorners = mNetworkTable.getEntry("tcornx").getDoubleArray(mZeroArray);
        double[] yCorners = mNetworkTable.getEntry("tcorny").getDoubleArray(mZeroArray);
        mSeesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;

        // something went wrong
        if (!mSeesTarget ||
                Arrays.equals(xCorners, mZeroArray) || Arrays.equals(yCorners, mZeroArray)
                || xCorners.length != 8 || yCorners.length != 8) {
            return null;
        }

        return extractTopCornersFromBoundingBoxes(xCorners, yCorners);
    }


    private static final Comparator<Translation2d> xSort = Comparator.comparingDouble(Translation2d::getX);
    private static final Comparator<Translation2d> ySort = Comparator.comparingDouble(Translation2d::getY);

    public static List<double[]> extractTopCornersFromBoundingBoxes(double[] xCorners, double[] yCorners) {
        List<Translation2d> corners = new ArrayList<>();
        for (int i = 0; i < xCorners.length; i++) {
            corners.add(new Translation2d(xCorners[i], yCorners[i]));
        }

        corners.sort(xSort);

        List<Translation2d> left = corners.subList(0, 4);
        List<Translation2d> right = corners.subList(4, 8);

        left.sort(ySort);
        right.sort(ySort);

        List<Translation2d> leftTop = left.subList(0, 2);
        List<Translation2d> rightTop = right.subList(0, 2);

        leftTop.sort(xSort);
        rightTop.sort(xSort);

        Translation2d leftCorner = leftTop.get(0);
        Translation2d rightCorner = rightTop.get(1);

        return List.of(new double[]{leftCorner.getX(), leftCorner.getY()}, new double[]{rightCorner.getX(), rightCorner.getY()});
    }

    public double getLatency() {
        return mPeriodicIO.latency;
    }

}

