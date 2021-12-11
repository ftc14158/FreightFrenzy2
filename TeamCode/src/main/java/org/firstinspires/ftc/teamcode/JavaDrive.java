package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * The main TeleOp drive command
 *
 * This command initializes the Robot container in TeleOp mode (so all the gamepad button
 * actions get set up, and the default command for the drivetrain is to follow the gamepad)
 *
 * It also sets the motors in RawPower mode to achieve maximum available power, without using
 * PID control to try and maintain a target velocity. When using the PID controller, the robot
 * does not drive as fast.
 *
 * This command also assumes the robot is starting with back against the center of the Red alliance wall,
 * facing the center of the field. Ideally, the robot could figure out where it is starting from using
 * vision instead.
 *
 */

@TeleOp(name = "Drive Robot (Java)")

public class JavaDrive extends CommandOpMode {

    private RobotContainer m_robot;

    @Override
    public void initialize() {
        m_robot = new RobotContainer( true, hardwareMap,
                gamepad1, gamepad2);

        m_robot.drivetrain.resetMotors( true );


        // set starting position - against center of red wall, facing middle of field

        m_robot.drivetrain.setPose( -0, -1.82, 90);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        schedule( new PerpetualCommand( new RunCommand( () -> { m_robot.sendTelem( dashboard );})));


    }
}
