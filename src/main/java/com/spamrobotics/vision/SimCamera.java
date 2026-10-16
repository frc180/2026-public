package com.spamrobotics.vision;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

/**
 * Simulates a Limelight 4 using PhotonVision.
 */
public class SimCamera {

    static final RawFiducial[] EMPTY_FIDUCIALS = new RawFiducial[0];

    final PhotonCamera camera;
    final PhotonPoseEstimator photonPoseEstimator;

    EasyPhotonvisionSimulator sim = null;

    public SimCamera(String name, Transform3d cameraPosition, AprilTagFieldLayout apriltagLayout) {
        this(name, () -> RobotContainer.instance.drivetrain.getSimPose(), cameraPosition, apriltagLayout);
    }

    public SimCamera(String name, Supplier<Pose2d> poseSupplier, Transform3d cameraPosition, AprilTagFieldLayout apriltagLayout) {
        camera = new PhotonCamera(name);
        photonPoseEstimator = new PhotonPoseEstimator(apriltagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraPosition);

        // Everything past this point is simulation only
        if (Robot.isReal()) return;

        sim = new EasyPhotonvisionSimulator(
            camera,
            cameraPosition,
            poseSupplier,
            EasyPhotonvisionSimulator.LL4_HAILO_1280_960(),
            apriltagLayout
        );
    }


    PoseEstimate poseEstimate = null;
    RawFiducial[] rawFiducials = null;
    double timestamp = 0;

    public void update() {
        sim.update();

        PhotonPipelineResult latestResult = null;
        EstimatedRobotPose latestEstimate = null;
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> poseEst = photonPoseEstimator.update(result);
            if (poseEst.isPresent()) {
                latestEstimate = poseEst.get();
                latestResult = result;
            }
        }
        List<PhotonTrackedTarget> targets = latestResult != null ? latestResult.getTargets() : null;

        poseEstimate = toPoseEstimate(latestEstimate, targets);
        if (poseEstimate != null) timestamp = poseEstimate.timestampSeconds;
        
        if (targets != null) {
            rawFiducials = toRawFiducials(targets);
        } else {
            rawFiducials = EMPTY_FIDUCIALS;
        }
    }

    public PoseEstimate getPoseEstimate() {
        return poseEstimate;
    }

    public RawFiducial[] getRawFiducials() {
        return rawFiducials;
    }

    /**
     * Converts a PhotonVision EstimatedRobotPose to a Limelight PoseEstimate
     */
    private PoseEstimate toPoseEstimate(EstimatedRobotPose estimate, List<PhotonTrackedTarget> targets) {
        if (estimate == null) return null;

        PoseEstimate est = new PoseEstimate();
        est.pose = estimate.estimatedPose.toPose2d();
        est.timestampSeconds = estimate.timestampSeconds;
        est.tagCount = targets.size();
        est.avgTagDist = targets.stream().mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getDistance(Translation3d.kZero)).average().orElse(0);
        est.avgTagArea = 99; //targets.stream().mapToDouble(t -> t.area).average().orElse(0);
        return est;
    }

    /**
     * Converts a PhotonTrackedTarget list to a Limelight RawFiducials array
     */
    private RawFiducial[] toRawFiducials(List<PhotonTrackedTarget> targets) {
        RawFiducial[] fiducials = new RawFiducial[targets.size()];
        for (int i = 0; i < targets.size(); i++) {
            PhotonTrackedTarget target = targets.get(i);
            double dist = target.getBestCameraToTarget().getTranslation().getDistance(Translation3d.kZero);

            fiducials[i] = new RawFiducial(
                target.getFiducialId(),
                target.getYaw(),
                target.getPitch(),
                target.area,
                dist,
                dist,
                target.poseAmbiguity
            );
        }
        return fiducials;
    }
}