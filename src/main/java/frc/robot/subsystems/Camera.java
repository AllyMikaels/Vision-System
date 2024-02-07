// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cameraserver.CameraServerShared;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Camera extends SubsystemBase {
  /** Creates a new Camera. */
  

  public Camera() {
    
  }
   public void camSubsystem() {
    
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
      
    };





   

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
 

  }
}

