package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.Robot;
import frc.robot.Constants;
import edu.wpi.first.math.Matrix;

public class Vision extends SubsystemBase {

    private PhotonCamera[] cameras;
    private PhotonPoseEstimator[] poseEstimators;
    private SwerveSubsystem mSwerveSubsystem;
    private Matrix<N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;
    private Pose2d estimatedPose;

    public Vision(EstimateConsumer estConsumer, SwerveSubsystem mSwerveSubsystem) {
        // Dynamically create cameras and pose estimators based on CAMERA_CONFIGS array
        int numCameras = CameraConstants.CAMERA_CONFIGS.length;
        cameras = new PhotonCamera[numCameras];
        poseEstimators = new PhotonPoseEstimator[numCameras];
        this.estConsumer = estConsumer;
        this.mSwerveSubsystem = mSwerveSubsystem;

        for (int i = 0; i < numCameras; i++) {
            cameras[i] = new PhotonCamera(CameraConstants.CAMERA_CONFIGS[i].name);
            poseEstimators[i] = new PhotonPoseEstimator(
                    CameraConstants.kTagLayout,
                    CameraConstants.CAMERA_CONFIGS[i].robotToCamera);
        }

        estimatedPose = Constants.SIM_START_POSE;
        curStdDevs = CameraConstants.kSingleTagStdDevs;
    }

    @Override
    public void periodic() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        // Process all cameras
        for (int i = 0; i < cameras.length; i++) {
            PhotonCamera camera = cameras[i];
            PhotonPoseEstimator estimator = poseEstimators[i];

            for (var result : camera.getAllUnreadResults()) {
                visionEst = estimator.estimateCoprocMultiTagPose(result);
                if (visionEst.isEmpty()) {
                    visionEst = estimator.estimateLowestAmbiguityPose(result);
                }
                updateEstimationStdDevs(visionEst, result.getTargets(), estimator);

                visionEst.ifPresent(
                        est -> {
                            // Change our trust in the measurement based on the tags we can see
                            var estStdDevs = getEstimationStdDevs();

                            estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                        });
            }
        }
    }

    public PhotonCamera[] getCameras() {
        return cameras;
    }

    public Pose2d getEstimatedPose() {
        return mSwerveSubsystem.getPose();
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates
     * dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from
     * the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets       All targets in this camera frame
     * @param estimator     The pose estimator for this camera
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets,
            PhotonPoseEstimator estimator) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = CameraConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = CameraConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            for (var tgt : targets) {
                var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = CameraConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = CameraConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 2)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 10));
                curStdDevs = estStdDevs;
            }
        }
    }

}
