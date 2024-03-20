package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import java.util.Optional;

public class Limelight extends SubsystemBase {
    private static Limelight instance;
    public SwerveDrivePoseEstimator swervePose;


    public static Limelight getInstance() {
        return Limelight.instance == null
                ? Limelight.instance = new Limelight()
                : Limelight.instance;
    }

    /**
     * Routinely sends debugging information to SmartDashboard
     */
    @Override
    public void periodic() {
        SmartDashboard.putString("Camera junk: ", LimelightHelpers.getBotPose2d_wpiBlue("limelight").toString());
        Pose2d pose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
        SmartDashboard.putString("Camera Results", "X: " + Drivetrain.getInstance().getPosition().getX() + " Y: " + Drivetrain.getInstance().getPosition().getY());
//        Optional<EstimatedRobotPose> pose = getRobotPose();
//        pose.ifPresent(estimatedRobotPose -> SmartDashboard.putString("Pose", estimatedRobotPose.estimatedPose.toString()));
//        pose.ifPresent(estimatedRobotPose -> SmartDashboard.putString("Distance: ", "" + distanceFromGoal(estimatedRobotPose.estimatedPose.toPose2d())));
//        pose.ifPresent(estimatedRobotPose -> SmartDashboard.putString("Closest point: ", getClosestScoringPoint(estimatedRobotPose).toString()));
//        SmartDashboard.putString("Relative angle: ", "" + angleRelativeToGoal());
//        SmartDashboard.putString("Score angle: ", angleToScore().toString());
//        SmartDashboard.putString("Get to score: ", getToScoringPosition().toString());


    }

    /**
     * Gets the distance from a goal based on the AprilTag data as viewed by the camera
     * @param pose The robot's current position FIXME: uncertain
     * @return The distance from a goal
     **/
    public double distanceFromGoal(Pose2d pose){
        return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)) {
            case Red -> {
                Transform2d transRed = new Transform2d(Constants.Camera.field.getTagPose(7).get().toPose2d(), pose);
                yield Math.sqrt(Math.pow(transRed.getX(), 2) + Math.pow(transRed.getY(), 2));
            }
            case Blue -> {
                Transform2d transBlue = new Transform2d(Constants.Camera.field.getTagPose(4).get().toPose2d(), pose);
                yield Math.sqrt(Math.pow(transBlue.getX(), 2) + Math.pow(transBlue.getY(), 2));
            }
        };
    }

    /**
     * Gets a coordinate point representing the closest scoring goal.
     * @return A Pose3d containing that coordinate
     *
     * In inches rn will change to meters later
     **/
    public Pose2d getClosestScoringPoint(Pose2d pose) {
        double xPoint = 0;
        double yPoint = 0;
        Rotation2d currentRotation = pose.getRotation();
        if (distanceFromGoal(pose) <= Constants.cameraRange) {
            return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)) {
                case Blue:
                    xPoint = Constants.scoringRange * Math.cos(angleRelativeToGoal(pose)) - Units.inchesToMeters(1.5);
                    yPoint = Constants.scoringRange * Math.sin(angleRelativeToGoal(pose)) + Units.inchesToMeters(218.42);
                    yield new Pose2d(xPoint, yPoint, currentRotation);
                case Red:
                    xPoint = Constants.scoringRange * Math.cos(angleRelativeToGoal(pose)) + Units.inchesToMeters(652.73);
                    yPoint = Constants.scoringRange * Math.sin(angleRelativeToGoal(pose)) + Units.inchesToMeters(218.42);
                    yield new Pose2d(xPoint, yPoint, currentRotation);
            };
        }
        return new Pose2d();
    }

    /**
     * Returns angle relative to the goal regardless of what way the robot is facing
     * @return The angle relative to the goal.
     *
     * In inches change to meters later
     **/
    public double angleRelativeToGoal(Pose2d pose) {
        return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)) {
            case Red -> Math.PI - Math.asin((pose.getY() - Units.inchesToMeters(218.42)) / Math.sqrt(
                    Math.pow(pose.getY() - Units.inchesToMeters(218.42), 2) + Math.pow(pose.getX() - Units.inchesToMeters(652.73), 2)
            ));
            case Blue -> (2 * Math.PI) + Math.asin((pose.getY() - Units.inchesToMeters(218.42)) / Math.sqrt(
                    Math.pow(pose.getY() - Units.inchesToMeters(218.42), 2) + Math.pow(pose.getX() + Units.inchesToMeters(1.5), 2)
            ));
        };
    }

    /**
     * Returns the angle that the robot needs to be facing to be lined up with the goal
     */
    public Rotation2d angleToScore(Pose2d pose) {
            switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)) {
                case Red -> {
                    return new Rotation2d(pose.getRotation().getRadians() - (angleRelativeToGoal(pose) - Math.PI));
                }
                case Blue -> {
                    return new Rotation2d(pose.getRotation().getRadians() - (angleRelativeToGoal(pose) + Math.PI));
                }
            };
        return null;
    }

    /**
     * Returns a Transform2d with the distance needed to move(if at all) and the rotation needed to face the goal
     * Can be adjusted to be in certain scoring spots rather than FIXME: unclear
     * @return The Transform2d object
     **/
    public Transform2d getToScoringPosition(Pose2d pose) {
        // If in scoring range assuming that the range has a radius
        if (distanceFromGoal(pose) <= Constants.scoringRange) {
            // Return Transform2d staying in same place and just rotating to line up with goal
            return new Transform2d(new Translation2d(), angleToScore(pose));
        } else if (distanceFromGoal(Drivetrain.getInstance().getPosition()) <= Constants.cameraRange) {
            // Hopefully returns a Transform2d that tells robot where to go and how much to rotate by
            return new Transform2d(
                    new Translation2d(Math.abs(getClosestScoringPoint(pose).getX() - pose.getX()),
                            Math.abs(getClosestScoringPoint(pose).getY() - pose.getY())),
                    angleToScore(pose)
            );
        }
        return null;
    }
}
