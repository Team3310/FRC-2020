package frc.robot.subsystems;

import java.util.List;

import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.robot.SubsystemManager;


/**
 * Class that manages using multiple Limelight 2's, one at a time
 * 
 * @see Limelight
 */
public class LimelightManager extends Subsystem {

    private static LimelightManager sInstance = null;
    private final Limelight mLimelight;
    private List<Limelight> mAllLimelights;

    enum ActiveLimelight {
        mLimelight
    }

    private final ActiveLimelight mActiveLimelight = ActiveLimelight.mLimelight;

    private LimelightManager() {
        mLimelight = new Limelight(Constants.kLimelightConstants);
    }

    public static LimelightManager getInstance() {
        if (sInstance == null) {
            sInstance = new LimelightManager();
        }
        return sInstance;
    }

    // @Override
    public void registerEnabledLoops(final ILooper mEnabledLooper) {
        final Loop mLoop = new Loop() {
            @Override
            public void onStart(final double timestamp) {
                mAllLimelights.forEach(limelight -> limelight.setLed(Limelight.LedMode.OFF));
                RobotState.getInstance().resetVision();
            }

            @Override
            public void onLoop(final double timestamp) {
                synchronized (LimelightManager.this) {
                    final Limelight limelight;

                    // if (mActiveLimelight == ActiveLimelight.mLimelight) {
                    //     RobotState.getInstance().addVisionUpdate(timestamp - limelight.getLatency(), null,
                    //             getActiveLimelightObject());
                    // } else {
                    //     RobotState.getInstance().addVisionUpdate(timestamp - limelight.getLatency(),
                    //             limelight.getTarget(), getActiveLimelightObject());
                    // }
                }

            }

            // @Override
            public void onStop(final double timestamp) {
                stop();
            }
        };

        mEnabledLooper.register(mLoop);
    }

    // @Override
    public synchronized void readPeriodicInputs() {
        mAllLimelights.forEach(limelight -> limelight.readPeriodicInputs());
    }

    // @Override
    public synchronized void writePeriodicOutputs() {
        mAllLimelights.forEach(limelight -> limelight.writePeriodicOutputs());
    }

    // @Override
    public synchronized void stop() {
        mAllLimelights.forEach(limelight -> limelight.stop());
    }

    // @Override
    public boolean checkSystem() {
        return true;
    }

    // @Override
    public void outputTelemetry() {
        mAllLimelights.forEach(limelight -> limelight.outputTelemetry());
    }

    public synchronized ActiveLimelight getActiveLimelight() {
        return mActiveLimelight;
    }

    public synchronized void setUseTopLimelight(final boolean useTop) {
        getInactiveLimelightObject().setLed(Limelight.LedMode.OFF);
        getActiveLimelightObject().setLed(Limelight.LedMode.PIPELINE);
    }

    private synchronized Limelight getActiveLimelightObject() {
        return mLimelight;
    }

    private synchronized Limelight getInactiveLimelightObject() {
        return mLimelight;
    }

    public synchronized void setPipeline(final int mode) {
        mAllLimelights.forEach(limelight -> limelight.setPipeline(mode));
    }

    public synchronized void triggerOutputs() {
        mAllLimelights.forEach(limelight -> limelight.triggerOutputs());
    }

    public synchronized void setAllLeds(final Limelight.LedMode mode) {
        mAllLimelights.forEach(limelight -> limelight.setLed(mode));
    }


}
