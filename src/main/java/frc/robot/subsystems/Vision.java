package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonCamera.*;
import org.photonvision.targeting.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    PhotonCamera camera = new PhotonCamera("photonvision");
    PhotonPipelineResult result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    List<PhotonTrackedTarget> targets = result.getTargets();

    public Vision() {

    }
}
