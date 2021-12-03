package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RamseteCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.AutonomousConstants;
import org.firstinspires.ftc.teamcode.command.PositionArm;
import org.firstinspires.ftc.teamcode.command.RamseteFollow;

import java.util.ArrayList;
import java.util.List;

/**
 * Simple test of autonomous driving using field positions and the
 * ramsete controller.
 */
@Autonomous(name="Drive Test")

public class AutoDriveTest extends CommandOpMode {
    private RobotContainer m_robot;

    @Override
    public void initialize() {

        // Create an FTC dashboard object to use to send back telemetry information
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        // Create the main robot object that contains the various
        // subsystems etc
        m_robot = new RobotContainer( false, hardwareMap,
                gamepad1);

        // Create a command that will run every time in the loop to send back the telemetry to
        // the FTC dashboard
        schedule( new PerpetualCommand( new RunCommand( () -> { m_robot.sendTelem( dashboard );})));

        // set starting position of robot
        m_robot.drivetrain.setPose(  AutonomousConstants.START_X, AutonomousConstants.START_Y, AutonomousConstants.START_HEADING);
        // keep resetting motors at the startto ensure no lingering timestamp

//        Log.w("DRIVETEST",  "Trajectory time = " + trajectory.getTotalTimeSeconds());


        // Create a command to drive to the target pose

        RamseteFollow x1 = new RamseteFollow(
                m_robot,
                new Pose2d( AutonomousConstants.TARGET_X, AutonomousConstants.TARGET_Y, Rotation2d.fromDegrees( AutonomousConstants.TARGET_HEADING ) ),
                new ArrayList<Translation2d>(),
                false
                );


        // Create another command to drive in reverse back to the starting pose
        RamseteFollow x2 = new RamseteFollow(
                m_robot,
                new Pose2d( AutonomousConstants.START_X, AutonomousConstants.START_Y, Rotation2d.fromDegrees(AutonomousConstants.START_HEADING) ),
                new ArrayList<Translation2d>(),
                true

        );

        // Schedule the two commands to happen one after the other, with some
        // dead commands for time delays in betweeen

        schedule( new SequentialCommandGroup(x1,
                new RunCommand( () -> {} ).withTimeout( 3000 ),
                x2,
                new RunCommand( () -> {} ).withTimeout( 5000 ),
                new InstantCommand( () -> { m_robot.drivetrain.resetMotors(false); }, m_robot.drivetrain )
                ) );


    }

}
