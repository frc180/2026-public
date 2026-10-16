package com.spamrobotics.vision;

import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Robot;

public class EasyPhotonvisionSimulator {
    
    VisionSystemSim visionSim = null;
    PhotonCameraSim cameraSim = null;
    Supplier<Pose2d> robotPoseSupplier = null;

    public EasyPhotonvisionSimulator(
        PhotonCamera camera,
        Transform3d cameraPosition,
        Supplier<Pose2d> robotPoseSupplier,
        Supplier<SimCameraProperties> cameraPropertiesSupplier,
        AprilTagFieldLayout apriltagLayout
    ) {
        this(
            camera,
            cameraPosition,
            robotPoseSupplier,
            cameraPropertiesSupplier.get(),
            apriltagLayout
        );
    }

    public EasyPhotonvisionSimulator(
        PhotonCamera camera,
        Transform3d cameraPosition,
        Supplier<Pose2d> robotPoseSupplier,
        SimCameraProperties cameraProperties,
        AprilTagFieldLayout apriltagLayout
    ) {
        if (Robot.isReal()) return;
        
        this.robotPoseSupplier = robotPoseSupplier;
        visionSim = new VisionSystemSim(camera.getName());
        visionSim.addAprilTags(apriltagLayout);

        cameraSim = new PhotonCameraSim(camera, cameraProperties);
        visionSim.addCamera(cameraSim, cameraPosition);

        cameraSim.enableDrawWireframe(true);
    }

    public void update() {
        if (visionSim != null) {
            visionSim.update(robotPoseSupplier.get());
        }
    }

    // https://docs.limelightvision.io/docs/docs-limelight/getting-started/hardware-comparison

    private static final Rotation2d LL4_FOV = diagonalFOV(87, 56); // was 100
    private static final Rotation2d LL3G_FOV = LL4_FOV;
    private static final Rotation2d LL3_FOV = diagonalFOV(62.5, 48.9);

    public static SimCameraProperties LL4_HAILO_1280_960() {
        return camera(1280, 960, LL4_FOV, 45);
    };

    public static SimCameraProperties LL4_1280_960() {
        SimCameraProperties props = LL4_HAILO_1280_960();
        props.setFPS(40);
        return props;
    }

    public static SimCameraProperties LL3G_1280_960() {
        return camera(1280, 960, LL3G_FOV, 20);
    }

    public static SimCameraProperties LL3G_640_480() {
        return camera(640, 480, LL3G_FOV, 50);
    }

    public static SimCameraProperties LL3_1280_960() {
        return camera(1280, 960, LL3_FOV, 20);
    }

    public static SimCameraProperties LL3_640_480() {
        return camera(640, 480, LL3_FOV, 50);
    }

    public static SimCameraProperties camera(int resWidth, int resHeight, Rotation2d diagonalFov, int fps) {
        SimCameraProperties props = baseCameraProperties();
        props.setCalibration(resWidth, resHeight, diagonalFov);
        props.setFPS(fps);
        return props;
    }

    private static SimCameraProperties baseCameraProperties() {
        SimCameraProperties props = new SimCameraProperties();
        props.setCalibError(0.35, 0.10);
        props.setAvgLatencyMs(50);
        props.setLatencyStdDevMs(15);
        return props;
    }
    
    private static Rotation2d diagonalFOV(double horizontalFOV, double verticalFOV) {
        return Rotation2d.fromDegrees(diagonalFOVDegrees(horizontalFOV, verticalFOV));
    }

    private static double diagonalFOVDegrees(double horizontalFOV, double verticalFOV) {
        return Math.toDegrees(Math.atan(Math.sqrt(
            Math.tan(Math.toRadians(horizontalFOV / 2)) * Math.tan(Math.toRadians(horizontalFOV / 2)) +
            Math.tan(Math.toRadians(verticalFOV / 2)) * Math.tan(Math.toRadians(verticalFOV / 2))
        )) * 2);
    }
}