package frc.lib.logging;

import org.opencv.core.Mat;
import org.opencv.videoio.VideoWriter;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;

public class VideoRecording {
    public static void startRecording() {
        // Create a USB camera object
        UsbCamera camera = CameraServer.startAutomaticCapture();

        // Set camera resolution and frame rate
        camera.setResolution(640, 480);
        camera.setFPS(2);

        // Create a CvSink object to grab frames from the camera
        CvSink cvSink = CameraServer.getVideo();

        // Create a Mat object to store the captured frame
        Mat image = new Mat();

        // Create a VideoWriter object to save the video to a file
        VideoWriter writer = new VideoWriter("output.avi",
                VideoWriter.fourcc('M', 'J', 'P', 'G'), 30,
                new org.opencv.core.Size(640, 480), true);

        // Create a loop to capture frames and save them to the file
        while (!Thread.interrupted()) {
            // Grab a frame from the camera
            cvSink.grabFrame(image);

            // Write the frame to the video file
            writer.write(image);
        }

        // Release the resources used by the video writer
        writer.release();
    }
}
