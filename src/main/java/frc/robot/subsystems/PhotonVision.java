package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;

public class PhotonVision extends SubsystemBase {
    private static PhotonVision instance;
    private PhotonCamera camera;
    private PhotonPipelineResult latestResult = null;
    private PhotonPoseEstimator photonPose;
    public SwerveDrivePoseEstimator swervePose;


    public static PhotonVision getInstance() {
        return PhotonVision.instance == null
                ? PhotonVision.instance = new PhotonVision()
                : PhotonVision.instance;
    }

    public PhotonVision() {
        this.camera = new PhotonCamera(Constants.Ports.CAMERA);
        photonPose = new PhotonPoseEstimator(
                Constants.Camera.field,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera,
                Constants.Camera.cameraToRobot
        );
    }

    /**
     * Routinely sends debugging information to SmartDashboard
     */
    @Override
    public void periodic() {
        SmartDashboard.putString("Camera junk: ", camera.getLatestResult().getTargets().toString());
        SmartDashboard.putString("Camera Results", "X: " + Drivetrain.getInstance().getPosition().getX() + " Y: " + Drivetrain.getInstance().getPosition().getY());
        Optional<EstimatedRobotPose> pose = getRobotPose();
        pose.ifPresent(estimatedRobotPose -> SmartDashboard.putString("Pose", estimatedRobotPose.estimatedPose.toString()));

    }

    /**
     * Gets current pose and timestamp on field using PhotonPoseEstimator
     **/
    public Optional<EstimatedRobotPose> getRobotPose(){
        return photonPose.update();
    }

    /**
     * Gets the distance from a goal based on the AprilTag data as viewed by the camera
     * @param pose The robot's current position FIXME: uncertain
     * @return The distance from a goal
     **/
    public double distanceFromGoal(Pose2d pose){
        return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)) {
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
    public Pose2d getClosestScoringPoint() {
        double xPoint = 0;
        double yPoint = 0;
        Optional<EstimatedRobotPose> pose = getRobotPose();
        Rotation2d currentRotation = null;
        if (pose.isPresent()) {
            currentRotation = pose.get().estimatedPose.getRotation().toRotation2d();
            if (distanceFromGoal(pose.get().estimatedPose.toPose2d()) <= Constants.cameraRange) {
                return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)) {
                    // Math will change when angleRelativeToGoal is finished
                    case Blue:
                        xPoint = Constants.scoringRange * Math.cos(angleRelativeToGoal()) - Units.inchesToMeters(1.5);
                        yPoint = Constants.scoringRange * Math.sin(angleRelativeToGoal()) + Units.inchesToMeters(218.42);
                        yield new Pose2d(xPoint, yPoint, currentRotation);
                    case Red:
                        xPoint = Constants.scoringRange * Math.cos(angleRelativeToGoal()) + Units.inchesToMeters(652.73);
                        yPoint = Constants.scoringRange * Math.sin(angleRelativeToGoal()) + Units.inchesToMeters(218.42);
                        yield new Pose2d(xPoint, yPoint, currentRotation);
                };
            }
        }
        return null;
    }

    /**
     * Returns angle relative to the goal regardless of what way the robot is facing
     * @return The angle relative to the goal.
     *
     * In inches change to meters later
     **/
    public double angleRelativeToGoal() {
        return switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)) {
            case Red -> Math.PI - Math.asin((Drivetrain.getInstance().getPosition().getY() - Units.inchesToMeters(218.42)) / Math.sqrt(
                    Math.pow(Drivetrain.getInstance().getPosition().getY() - Units.inchesToMeters(218.42), 2) + Math.pow(Drivetrain.getInstance().getPosition().getX() - Units.inchesToMeters(652.73), 2)
            ));
            case Blue -> (2 * Math.PI) + Math.asin((Drivetrain.getInstance().getPosition().getY() - Units.inchesToMeters(218.42)) / Math.sqrt(
                    Math.pow(Drivetrain.getInstance().getPosition().getY() - Units.inchesToMeters(218.42), 2) + Math.pow(Drivetrain.getInstance().getPosition().getX() + Units.inchesToMeters(1.5), 2)
            ));
        };
    }

    /**
     * Returns the angle that the robot needs to be facing to be lined up with the goal
     */
    public Rotation2d angleToScore() {
        Optional<EstimatedRobotPose> pose = getRobotPose();
        if(pose.isPresent()) {
            switch (DriverStation.getAlliance().orElse(DriverStation.Alliance.Red)) {
                case Red -> {
                    return new Rotation2d(pose.get().estimatedPose.getRotation().toRotation2d().getRadians() - (angleRelativeToGoal() - Math.PI));
                }
                case Blue -> {
                    return new Rotation2d(pose.get().estimatedPose.getRotation().toRotation2d().getRadians() - (angleRelativeToGoal() + Math.PI));
                }
            };
        };
        return null;
    }

    /**
     * Returns a Transform2d with the distance needed to move(if at all) and the rotation needed to face the goal
     * Can be adjusted to be in certain scoring spots rather than FIXME: unclear
     * @return The Transform2d object
     **/
    public Transform2d getToScoringPosition() {
        // If in scoring range assuming that the range has a radius
        if (distanceFromGoal(Drivetrain.getInstance().getPosition()) <= Constants.scoringRange) {
            // Return Transform2d staying in same place and just rotating to line up with goal
            return new Transform2d(new Translation2d(), angleToScore());
        } else if (distanceFromGoal(Drivetrain.getInstance().getPosition()) <= Constants.cameraRange) {
            // Hopefully returns a Transform2d that tells robot where to go and how much to rotate by
            return new Transform2d(
                    new Translation2d(Math.abs(getClosestScoringPoint().getX() - Drivetrain.getInstance().getPosition().getX()),
                            Math.abs(getClosestScoringPoint().getY() - Drivetrain.getInstance().getPosition().getY())),
                    angleToScore()
            );
        }
        return null;
    }

    public PhotonPipelineResult getLatestResult(){
        return camera.getLatestResult();
    }
}
