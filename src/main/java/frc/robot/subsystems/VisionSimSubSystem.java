/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package frc.robot.subsystems;

import static frc.robot.Constants.CameraConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import frc.robot.Robot;
import frc.robot.Constants.CameraConstants;

public class VisionSimSubSystem extends SubsystemBase {
    private Vision mVision;
    // Simulation
    private PhotonCameraSim[] cameraSims;
    private VisionSystemSim visionSim;

    public VisionSimSubSystem(Vision mVision) {
        PhotonCamera[] cams = mVision.getCameras();
        // Create the vision system simulation which handles cameras and targets on the
        // field.
        visionSim = new VisionSystemSim("main");
        // Add all the AprilTags inside the tag layout as visible targets to this
        // simulated field.
        visionSim.addAprilTags(kTagLayout);

        // Initialize cameraSims array
        cameraSims = new PhotonCameraSim[cams.length];

        // Set up simulation for each camera dynamically
        for (int i = 0; i < cams.length; i++) {
            // Create simulated camera properties. These can be set to mimic your actual
            // camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(320, 240, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(70);
            cameraProp.setAvgLatencyMs(30);
            cameraProp.setLatencyStdDevMs(10);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values
            // with visible targets.
            var camSim = new PhotonCameraSim(cams[i], cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            // Use the corresponding camera config from Constants
            visionSim.addCamera(camSim, CameraConstants.CAMERA_CONFIGS[i].robotToCamera);
            camSim.enableDrawWireframe(true);
            cameraSims[i] = camSim;
        }

        this.mVision = mVision;
        resetSimPose();
    }

    @Override
    public void simulationPeriodic() {
        visionSim.update(mVision.getEstimatedPose());
    }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose() {
        Pose2d startingPose = new Pose2d(new Translation2d(1,
                4),
                Rotation2d.fromDegrees(0));
        visionSim.resetRobotPose(startingPose);
    }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation())
            return null;
        return visionSim.getDebugField();
    }
}