package frc.robot.auto;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants;

import java.util.List;

public class TrajectoryGenerator {
    private static final TrajectoryGenerator instance = new TrajectoryGenerator();

    public static TrajectoryGenerator getInstance() {
        return instance;
    }

    // Create a voltage constraint to ensure we don't accelerate too fast
    DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(Constants.ksVolts,
                            Constants.kvVoltSecondsPerMeter,
                            Constants.kaVoltSecondsSquaredPerMeter),
                    Constants.kDriveKinematics,
                    10);

    // Create config for trajectory
    TrajectoryConfig forwardConfig =
            new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                    Constants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint);

    TrajectoryConfig reverseConfig =
            new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                    Constants.kMaxAccelerationMetersPerSecondSquared)
                    // Add kinematics to ensure max speed is actually obeyed
                    .setKinematics(Constants.kDriveKinematics)
                    // Apply the voltage constraint
                    .addConstraint(autoVoltageConstraint)
                    .setReversed(true);

        public Trajectory getDriveStraight(){
            return edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0),
                            new Rotation2d(Units.radiansToDegrees(0))),
                    List.of(
                            new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0))
                    ),
                    new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(0),
                            new Rotation2d(Units.radiansToDegrees(0))),
                    // Pass config
                    forwardConfig
            );
        }

        public Trajectory getDriveStraightReversed(){
            return edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(120), Units.inchesToMeters(0),
                            new Rotation2d(Units.radiansToDegrees(0))),
                    List.of(
                            new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0))
                    ),
                    new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0),
                            new Rotation2d(Units.radiansToDegrees(0))),
                    // Pass config
                    reverseConfig
            );
        }

        //Start 8 Ball Trench Auto
        public Trajectory getCenterStartToEndOfTrench() {
            Trajectory centerStartToEndOfTrench;
            centerStartToEndOfTrench = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)),
                    List.of(
                            new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(40)),
                            new Translation2d(Units.inchesToMeters(80), Units.inchesToMeters(60)),
                            new Translation2d(Units.inchesToMeters(160), Units.inchesToMeters(60)),
                            new Translation2d(Units.inchesToMeters(240), Units.inchesToMeters(60))
                    ),
                    new Pose2d(Units.inchesToMeters(300), Units.inchesToMeters(66), new Rotation2d(0)),
                    // Pass config
                    forwardConfig
            );
            return centerStartToEndOfTrench;
        }

    public Trajectory getEndOfTrenchToStartOfTrench() {
            Trajectory endOfTrenchToStartOfTrench;
            endOfTrenchToStartOfTrench = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(300), Units.inchesToMeters(66), new Rotation2d(0)),
                    List.of(
                            new Translation2d(Units.inchesToMeters(180), Units.inchesToMeters(64))
                    ),
                    new Pose2d(Units.inchesToMeters(80), Units.inchesToMeters(50), new Rotation2d(0)),
                    // Pass config
                    reverseConfig
            );
            return endOfTrenchToStartOfTrench;
        }
        //End 8 Ball Trench Auto

        //Start 5 Ball Steal Auto
        public Trajectory getStealStartToStealBall() {
            Trajectory stealStartToStealSpot;
            stealStartToStealSpot = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)),
                    List.of(
                            new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(0))
                    ),
                    new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(0), new Rotation2d(0)),
                    // Pass config
                    forwardConfig
            );
            return stealStartToStealSpot;
        }

        public Trajectory getStealBallToCenterShot() {
            Trajectory stealSpotToCenterShot;
            stealSpotToCenterShot = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(150), Units.inchesToMeters(0), new Rotation2d(0)),
                    List.of(
                            new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60))
                    ),
                    new Pose2d(Units.inchesToMeters(-20), Units.inchesToMeters(190), new Rotation2d(0)),
                    // Pass config
                    reverseConfig
            );
            return stealSpotToCenterShot;
        }
        //End 5 Ball Steal Auto

        //Start 10 Ball Rendezvous/Trench Auto
        public Trajectory getCenterStartToRendezvous2ball() {
            Trajectory centerStartToRendezvous2Ball;
            centerStartToRendezvous2Ball = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)),
                    List.of(
                            new Translation2d(Units.inchesToMeters(80), Units.inchesToMeters(40))
                    ),
                    new Pose2d(Units.inchesToMeters(130), Units.inchesToMeters(-10), new Rotation2d(Units.degreesToRadians(-60))),
                    // Pass config
                    forwardConfig
            );
            return centerStartToRendezvous2Ball;
        }

        public Trajectory getRendezvous2BallToStartOfTrench() {
            Trajectory rendezvous2BallToStartOfTrench;
            rendezvous2BallToStartOfTrench = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(130), Units.inchesToMeters(-10), new Rotation2d(Units.degreesToRadians(-60))),
                    List.of(
                            new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(30))
                    ),
                    new Pose2d(Units.inchesToMeters(70), Units.inchesToMeters(65), new Rotation2d(Units.degreesToRadians(0))),
                    // Pass config
                    reverseConfig
            );
            return rendezvous2BallToStartOfTrench;
        }

        public Trajectory getStartOfTrenchToEndOfTrench() {
            Trajectory startOfTrenchToEndOfTrench;
            startOfTrenchToEndOfTrench = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(70), Units.inchesToMeters(65), new Rotation2d(0)),
                    List.of(
                            new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(60))
                    ),
                    new Pose2d(Units.inchesToMeters(216), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(0))),
                    // Pass config
                    forwardConfig
            );
            return startOfTrenchToEndOfTrench;
        }

        public Trajectory getEndOfTrenchToStartOfTrenchShot(){
            Trajectory endOfTrenchToStartOfTrenchShot;
            endOfTrenchToStartOfTrenchShot = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(216), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(0))),
                    List.of(
                            new Translation2d(Units.inchesToMeters(100), Units.inchesToMeters(60))
                    ),
                    new Pose2d(Units.inchesToMeters(82), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(0))),
                    // Pass config
                    reverseConfig
            );
            return endOfTrenchToStartOfTrenchShot;
        }
        //End 10 Ball Rendezvous/Trench Auto

        //Start 3 Ball Safe Auto
        public Trajectory getLeftStartToSafe(){
            Trajectory leftStartToSafe;
            leftStartToSafe = edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator.generateTrajectory(
                    new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))),
                    List.of(
                            new Translation2d(Units.inchesToMeters(-40), Units.inchesToMeters(0))
                    ),
                    new Pose2d(Units.inchesToMeters(-60), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))),
                    // Pass config
                    reverseConfig
            );
            return leftStartToSafe;
        }
        //End 3 Ball Safe Auto
    }
//}
