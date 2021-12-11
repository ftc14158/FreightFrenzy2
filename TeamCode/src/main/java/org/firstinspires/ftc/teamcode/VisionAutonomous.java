package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.StartEndCommand;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;
import org.firstinspires.ftc.teamcode.command.JerkRobotCommand;
import org.firstinspires.ftc.teamcode.command.PositionArm;
import org.firstinspires.ftc.teamcode.command.RamseteFollow;
import org.firstinspires.ftc.teamcode.vision.BarcodeDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;
import java.util.List;


public class VisionAutonomous extends CommandOpMode {
    private RobotContainer m_robot;

    private BarcodeDetectionPipeline pipelineBarcode;

    protected int startDelay = 10;

    protected boolean storageUnitEnd = false;

    private Pose2d m_homePose;
    private Pose2d m_hubTargetPose;
    private Pose2d m_warehouseParkPose;

    private List<Translation2d> warehouseWayPoints = new ArrayList<Translation2d>();
    private List<Translation2d> hubWayPoints = new ArrayList<Translation2d>();

    private OpenCvCamera camera;
    /**
     * Determine where we are starting from based on what is visible
     * and where to go for hub
     *
     */
    private void setupPoses() {
        double x, y, heading;
        double hub_x, hub_y, hub_heading;

        boolean isRed = pipelineBarcode.isRedVisible();
        boolean isStorageUnit = pipelineBarcode.isStorageUnitVisible();

        if (isRed) {
            heading = 90;   // facing out from wall
            y = -1.8;

            hub_y = -1.25;
        }
        else {   // blue side
            heading = 270;
            y = 1.8;

            hub_y = 1.25;
        }

        if (isStorageUnit) {
            x = -0.91;   // meters
            hub_x = -0.7;

            hub_heading = isRed ? 50 : 310;
        } else {
            x = 0.305;
            hub_x = 0.1;

            hub_heading = isRed ? 130 : 230;
        }


        m_hubTargetPose = new Pose2d(hub_x, hub_y, Rotation2d.fromDegrees(hub_heading));

        Log.i("VISION", "Start = " + x + "," + y + "," + heading);
        Log.i("VISION", "Hub = " + hub_x + "," + hub_y + "," + hub_heading);

        if (storageUnitEnd) {
            m_warehouseParkPose = new Pose2d(-1.4, isRed ? -0.9 : 0.9, Rotation2d.fromDegrees(180));
        } else {
            m_warehouseParkPose = new Pose2d(1.2, isRed ? -1.4 : 1.4, Rotation2d.fromDegrees(0));
        }

        Log.i("VISION", "Warehouse = " + m_warehouseParkPose.getX() + "," + m_warehouseParkPose.getY() + "," + m_warehouseParkPose.getHeading());
        if (isStorageUnit) {
            m_homePose = new Pose2d(x, y, Rotation2d.fromDegrees(heading));
        } else {
            m_homePose = new Pose2d(0, isRed ? -1.5 : 1.5, Rotation2d.fromDegrees(isRed ? 35 : 325) );
        }

        m_robot.drivetrain.setPose(x, y, heading);

        camera.stopStreaming(); // not needed any more
    }


    @Override
    public void initialize() {

        // DOn't initialize robot hardware at all for now
//        m_robot = new RobotContainer( false, hardwareMap,
 //               gamepad1);

        // Set up the camera to run with a viewer on the robot controller

        // Start by getting an ID to a viewer on the robot controller phone screen.

   //     int cameraMonitorViewId = hardwareMap.appContext.getResources()
   //             .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Build a camera object instance using the camera factory

        // Get the name of the webcam in the hardware configuration if we are using
        // an external webcam
 //       WebcamName webcamName = hardwareMap.get(WebcamName.class, "NAME_OF_CAMERA_IN_CONFIG_FILE");

        // Get OpenCV to create an object for the internal phone camera, attached to viewport..
        camera = OpenCvCameraFactory.getInstance()
                .createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK); // , cameraMonitorViewId);

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

                FtcDashboard dashboard = FtcDashboard.getInstance();
                Telemetry dashboardTelemetry = dashboard.getTelemetry();

                // Create the main robot object that contains the various
                // subsystems etc
                m_robot = new RobotContainer( false, hardwareMap,
                        gamepad1, gamepad2);

                // Create a command that will run every time in the loop to send back the telemetry to
                // the FTC dashboard
                schedule( new PerpetualCommand( new RunCommand( () -> { m_robot.sendTelem( dashboard );})));

                // set starting position of robot
//                m_robot.drivetrain.setPose(  Constants.AutonomousConstants.START_X, Constants.AutonomousConstants.START_Y, Constants.AutonomousConstants.START_HEADING);

                List<Translation2d> wayPoints = new ArrayList<Translation2d>();
                // wayPoints.add( new Translation2d(-0.6, 1 ) );

                RamseteFollow driveToHub = new RamseteFollow(
                        m_robot,
                        () -> m_hubTargetPose,  // new Pose2d( Constants.AutonomousConstants.TARGET_X, Constants.AutonomousConstants.TARGET_Y, Rotation2d.fromDegrees( Constants.AutonomousConstants.TARGET_HEADING ) ),
                        hubWayPoints,
                        false,
                        false
                );


                RamseteFollow driveToWarehouse = new RamseteFollow(
                        m_robot,
                        () -> m_warehouseParkPose,  // new Pose2d( Constants.AutonomousConstants.TARGET_X, Constants.AutonomousConstants.TARGET_Y, Rotation2d.fromDegrees( Constants.AutonomousConstants.TARGET_HEADING ) ),
                        warehouseWayPoints,
                        false,
                        true
                );
                // Create another command to drive in reverse back to the starting pose
                RamseteFollow driveHome = new RamseteFollow(
                        m_robot,
                        () -> m_homePose, // arehouseParkPose,
//                        new Pose2d( m_Constants.AutonomousConstants.START_X, Constants.AutonomousConstants.START_Y, Rotation2d.fromDegrees(Constants.AutonomousConstants.START_HEADING) ),
//                        new Pose2d( -1.4, 1.8, Rotation2d.fromDegrees(315) ),
                        new ArrayList<Translation2d>(),
                        true,
                        false

                );

                // Schedule the two commands to happen one after the other, with some
                // dead commands for time delays in betweeen

                schedule( new SequentialCommandGroup(
                        new FunctionalCommand( () -> {},
                                () -> setupPoses(),
                                (b) -> {},
                                () -> true,
                                m_robot.drivetrain
                                ),

                        new RunCommand( () -> {} ).withTimeout( startDelay ),
                        new JerkRobotCommand(m_robot),

//                        new RunCommand( () -> {}).withTimeout(500),
                        new RunCommand( () -> m_robot.intake.suck(), m_robot.intake ).withTimeout(1000),
                        new InstantCommand( () -> m_robot.intake.stop() ),
                        new ParallelCommandGroup(
                                driveToHub,
                                new InstantCommand(
                                        () -> m_robot.arm.goToPosition(
                                                ( pipelineBarcode.duckPos() == 1 ?
                                                (int)Constants.ArmConstants.POSITION1
                                        : (
                        pipelineBarcode.duckPos() == 2 ?
                                (int)Constants.ArmConstants.POSITION2 : (int)Constants.ArmConstants.POSITION3
                )

                        ) ) )
                        ),

                        new RunCommand( () -> m_robot.intake.eject(), m_robot.intake).withTimeout(3000),
                        new InstantCommand( () -> m_robot.intake.stop() ),
                        new ParallelCommandGroup(
                                driveHome,

                                new SequentialCommandGroup(
                                        new RunCommand( () -> {} ).withTimeout(1000),
                                        new InstantCommand(
                                        () -> m_robot.arm.setPower(0) )
                                )
                        ),
//                        new RunCommand( () -> {} ).withTimeout( 1000 ),
                        new ParallelCommandGroup(
                                driveToWarehouse,

                                new SequentialCommandGroup(
                                        new RunCommand( () -> {} ).withTimeout(1000),
                                        new InstantCommand(
                                                () -> m_robot.arm.goToLevel(1) )
                                )
                        ),

new InstantCommand( () -> requestOpModeStop() )



//                        new InstantCommand( () -> { m_robot.drivetrain.resetMotors(false); }, m_robot.drivetrain )
                ) );

            }


            @Override
            public void onError(int errorCode) {
                Log.e("OPENCV", "Camera did not open. error code = " + errorCode);

            }
        });


        // end of initialize.. op mode proceeds to execute nothing till stopped..
    }

}
