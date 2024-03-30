package org.carlmontrobotics.subsystems;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.UsbCameraInfo;
import edu.wpi.first.util.PixelFormat;
public class Camera {
    public Camera(){
        UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
        try (MjpegServer mjpegServer1 = new MjpegServer("serve_USB Camera 0", 1181)) {
            mjpegServer1.setSource(usbCamera);
        }
    }
    
}
