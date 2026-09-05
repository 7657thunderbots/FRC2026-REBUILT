package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonCamera.*;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.targeting.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.math.Matrix;

public class VisionSubsystem extends SubsystemBase {
    public static final AprilTagFieldLayout m_fieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2026RebuiltAndymark);

    public VisionSystemSim m_visionSim;

    private final double p_maximumAmbiguity = 0.25;
    private PhotonPoseEstimator p_poseEstimator;
    private PhotonCameraSim cameraSim;
    private double p_longDistancePoseEstimationCount = 0;
    private Supplier<Pose2d> p_currentPose;
    private Field2d p_field2d;
    private Matrix<N3, N1> curStdDevs;
    private Optional<EstimatedRobotPose> p_visionEstimate = Optional.empty();
    private EstimateConsumer p_estimateConsumer;

    private final PhotonCamera p_camera = new PhotonCamera("RearCamera");

    // Grab our back camera transform from the constants file
    private final Transform3d p_backCameraTransform = Constants.CameraConstants.CAMERA_CONFIGS[0].robotToCamera;

    public VisionSubsystem(Supplier<Pose2d> currentPose, Field2d field, EstimateConsumer estimateConsumer) {
        this.p_currentPose = currentPose;
        this.p_field2d = field;

        this.p_poseEstimator = new PhotonPoseEstimator(m_fieldLayout, p_backCameraTransform);
        this.p_estimateConsumer = estimateConsumer;

        if (Robot.isSimulation()) {
            m_visionSim = new VisionSystemSim("VisionSim");
            m_visionSim.addAprilTags(m_fieldLayout);
            // Create simulated camera properties. These can be set to mimic your actual
            // camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(320, 240, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(70);
            cameraProp.setAvgLatencyMs(30);
            cameraProp.setLatencyStdDevMs(10);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values
            // with visible
            // targets.
            cameraSim = new PhotonCameraSim(p_camera, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            m_visionSim.addCamera(cameraSim, p_backCameraTransform);

            cameraSim.enableDrawWireframe(true);
        }

    }

    public void UpdateAprilTagPositions() {
        List<PhotonPipelineResult> results = p_camera.getAllUnreadResults();

        if (results.isEmpty()) {
            return;
        }

        for (PhotonPipelineResult currentResult : results) {
            p_visionEstimate = p_poseEstimator.estimateCoprocMultiTagPose(currentResult);
            if (p_visionEstimate.isEmpty()) {
                p_visionEstimate = p_poseEstimator.estimateLowestAmbiguityPose(currentResult);
            }

            UpdateEstimationStandardDeviations(p_visionEstimate, currentResult.getTargets(), p_poseEstimator);

            p_visionEstimate.ifPresent(
                    est -> {
                        var estStdDevs = getEstimationStdDevs();
                        p_estimateConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                    });
        }

    }

    @Override
    public void periodic() {
        UpdateAprilTagPositions();
    }

    @Override
    public void simulationPeriodic() {
        m_visionSim.update(p_currentPose.get());
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
    private void UpdateEstimationStandardDeviations(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets,
            PhotonPoseEstimator estimator) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = CameraConstants.SINGLE_TAG_STD_DEVS;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = CameraConstants.SINGLE_TAG_STD_DEVS;
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
                curStdDevs = CameraConstants.SINGLE_TAG_STD_DEVS;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = CameraConstants.MULTI_TAG_STD_DEVS;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 2)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 10));
                curStdDevs = estStdDevs;
            }
        }
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

}
