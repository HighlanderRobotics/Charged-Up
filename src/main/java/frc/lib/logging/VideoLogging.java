package frc.lib.logging;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

import java.time.LocalDate;
import java.time.format.DateTimeFormatter;

import org.opencv.core.Mat;
import org.opencv.videoio.VideoWriter;

public class VideoLogging {

    public final static VideoLogging shared = new VideoLogging();

    private VideoLogging() { }

    public boolean isRecording = false;

    private UsbCamera camera;
    private CvSink cvSink;
    private VideoWriter writer;
    private DateTimeFormatter dateFormatter = DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm");
    /** 
    *Starts a Recording Capture and saves it to a specified filepath
    * @param filename The name of the file to save the video to
    * @param width The width of the video
    * @param height The height of the video
    * @param fps The frame rate of the video
     */
    public void startRecording(String filename, int width, int height, int fps) {
        isRecording = true;

        // Create a USB camera object
        camera = CameraServer.startAutomaticCapture();

        // Set camera resolution and frame rate
        camera.setResolution(width, height);
        camera.setFPS(fps);

        // Create a CvSink object to grab frames from the camera
        cvSink = CameraServer.getVideo();

            // Create a Mat object to store the captured frame
            Mat image = new Mat();

        // Create a VideoWriter object to save the video to a file
        writer = new VideoWriter(filename, VideoWriter.fourcc('M', 'J', 'P', 'G'), fps, new org.opencv.core.Size(width, height), true);

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

    /** 
     * An Overload for starting a recording capture
     * Filename is set to a formatted date
     * Width and Height are set to 640x480
     * FPS is set to 2
     */
    public void startRecording() {
        // convert date to string with pattern yyyy-MM-dd_HH-mm
        LocalDate localDate = LocalDate.now();

        startRecording(localDate.format(dateFormatter) + ".avi", 640, 480, 2);
    }

    public void stopRecording() {
        isRecording = false;

        // Interrupt the loop in the startRecording() method
        Thread.currentThread().interrupt();
    }
}
