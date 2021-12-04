package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;
import org.firstinspires.ftc.teamcode.command.PositionArm;
import org.firstinspires.ftc.teamcode.vision.BarcodeDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Vision Test")

public class VisionTest extends CommandOpMode {
    private RobotContainer m_robot;

    private BarcodeDetectionPipeline pipelineBarcode;

    @Override
    public void initialize() {

        // DOn't initialize robot hardware at all for now
//        m_robot = new RobotContainer( false, hardwareMap,
 //               gamepad1);

        // Set up the camera to run with a viewer on the robot controller

        // Start by getting an ID to a viewer on the robot controller phone screen.
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Build a camera object instance using the camera factory

        // Get the name of the webcam in the hardware configuration if we are using
        // an external webcam
 //       WebcamName webcamName = hardwareMap.get(WebcamName.class, "NAME_OF_CAMERA_IN_CONFIG_FILE");

        // Get OpenCV to create an object for the internal phone camera, attached to viewport..
        OpenCvCamera camera = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // Get OpenCV to create the camera object, with preview attached to the viewport for external webcam
//        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // open the camera device (synchronously for now)
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // start camera streaming (to viewport)
                Log.i("OPENCV", "Starting streaming at 320x240");

                // Use upright if phone is in portrait.. change to SIDEWAYS_LEFT / SIDEWAYS_RIGHT for landscape
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);


                // Should send camera stream to dashboard, but crashes..
                //FtcDashboard.getInstance().startCameraStream( camera, 0 );

                // now attach a pipeline to the video stream
                pipelineBarcode = new BarcodeDetectionPipeline( telemetry );
                camera.setPipeline( pipelineBarcode );
            }

            @Override
            public void onError(int errorCode) {
                Log.e("OPENCV", "Camera did not open. error code = " + errorCode);

            }
        });


        // end of initialize.. op mode proceeds to execute nothing till stopped..
    }

}
