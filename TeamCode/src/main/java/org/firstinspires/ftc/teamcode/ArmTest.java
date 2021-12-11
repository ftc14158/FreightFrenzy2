package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.command.PositionArm;

@Autonomous(name="Arm Test")

public class ArmTest extends CommandOpMode {
    private RobotContainer m_robot;

    @Override
    public void initialize() {
        m_robot = new RobotContainer( false, hardwareMap,
                gamepad1, gamepad2);

        schedule( new InstantCommand().withTimeout( 1000 )
                .andThen( new PositionArm(m_robot.arm, 200) )

         );

    }


}
