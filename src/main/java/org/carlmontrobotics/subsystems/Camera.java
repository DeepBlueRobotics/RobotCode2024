package org.carlmontrobotics.subsystems;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cscore.UsbCameraInfo;
import edu.wpi.first.util.PixelFormat;
public class Camera {
    public Camera(){
        UsbCamera usbCamera = new UsbCameraInfo("USB Camera 0", 0);
        MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181);
        mjpegServer1.setSource(usbCamera);

        // Creates the CvSink and connects it to the UsbCamera
        CvSink cvSink = new CvSink("opencv_USB Camera 0");
        cvSink.setSource(usbCamera);

        // Creates the CvSource and MjpegServer [2] and connects them
        CvSource outputStream = new CvSource("Blur", PixelFormat.kMJPEG, 640, 480, 30);
        MjpegServer mjpegServer2 = new MjpegServer("serve_Blur", 1182);
        mjpegServer2.setSource(outputStream);

    }
    
}
