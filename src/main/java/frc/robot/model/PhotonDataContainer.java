package frc.robot.model;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PhotonDataContainer {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    public PhotonDataContainer(PhotonCamera camera,
            PhotonPoseEstimator poseEstimator) {
        this.camera = camera;
        this.poseEstimator = poseEstimator;
    }

    public PhotonPoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public PhotonCamera getCamera() {
        return camera;
    }

}
