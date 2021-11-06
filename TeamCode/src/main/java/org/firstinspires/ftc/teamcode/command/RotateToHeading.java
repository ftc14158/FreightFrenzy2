package org.firstinspires.ftc.teamcode.command;

import android.util.Log;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.MathUtils;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.RefCounted;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IMU;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

public class RotateToHeading extends CommandBase {

    private Drivetrain m_drivetrain;
    private double m_targetHeading;
    private IMU m_imu;

    private PIDFController m_pid;

    private Telemetry m_telemetry;

    public RotateToHeading(Drivetrain drivetrain, IMU imu, double heding, Telemetry tel) {
        m_drivetrain = drivetrain;
        m_targetHeading = heding;
        m_imu = imu;
        m_telemetry = tel;

        m_pid = new PIDFController(DriveConstants.ROTATE_P,
                DriveConstants.ROTATE_I,
                DriveConstants.ROTATE_D,
                DriveConstants.ROTATE_F);

        Log.w("ROTATE-TO-HEADING", m_pid.getP() + "," + m_pid.getI() + "," + m_pid.getD() + "," + m_pid.getF() );

        m_pid.setSetPoint( m_targetHeading );
        m_pid.setTolerance( DriveConstants.ROTATE_TOLERANCE );
    }

    @Override
    public void execute() {
        double error, power;  //  = m_imu.getHeading() - m_targetHeading;

        error = m_pid.calculate( m_imu.getHeading() );

        m_telemetry.addData("Heading Error", error);

        //   90  * 1/90  = 1

//        error = error * (1.0 / 20.0 );

        // dont let the error ever be bigger than -1 to 1 because thats the
        // maximum sopeed of the motor
        power = clamp(error, -1.0, 1.0);

        Log.w("ROTATE-TO-HEADING", "Heading = " + m_imu.getHeading() + ", target=" + m_pid.getSetPoint() + ", error=" + error + ", power=" + power );

        m_drivetrain.drive( 0, 0, -power, false);

    }

    @Override
    public boolean isFinished() {

        return (  m_pid.atSetPoint() );  //  Math.abs(m_imu.getHeading() - m_targetHeading ) < 2 );
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.drive(0,0,0,false);
    }
}
