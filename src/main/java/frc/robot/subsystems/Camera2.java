// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//Web URL: http://photonvision.local:5800/#/dashboard

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera2 extends SubsystemBase {
  /* Creates a new Camera2. */
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

 /* public void camSubsystem() {
    
      Thread m_visionThread = new Thread(
        () -> {
        UsbCamera cam = CameraServer.startAutomaticCapture();
        cam.setResolution(640, 480);
        CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);
        CvSink cvSink = CameraServer.getVideo();
        Mat mat = new Mat();
        while (!Thread.interrupted()) {
          if (cvSink.grabFrame(mat) == 0) {
            outputStream.notifyError(cvSink.getError());
            continue;
        }
        Imgproc.rectangle(
           mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
      }
      outputStream.putFrame(mat);
      });
      m_visionThread.setDaemon(true);
      m_visionThread.start();
      
    };*/
  
PhotonCamera camera = new PhotonCamera("photonvision");
Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); 

PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);
  public Camera2() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = camera.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    double yaw = target.getYaw();
    double pitch = target.getPitch();
    double skew = target.getSkew();
    
    Transform3d pose = target.getBestCameraToTarget();
    int targetID = target.getFiducialId();
    double poseAmbiguity = target.getPoseAmbiguity();
   /* Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(targetID), pose);
    double distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, pose);
  Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
  distanceMeters, Rotation2d.fromDegrees(-target.getYaw()));*/
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update();
    }

    

}
