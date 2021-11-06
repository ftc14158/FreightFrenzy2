package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IMU extends SubsystemBase {
    private GyroEx m_gyro;
    private Telemetry m_telemetry;

    private double m_heading, m_absoluteHeading;
    private Rotation2d m_rotation2d;

    public IMU(GyroEx gyro, Telemetry telemetry) {
        m_gyro = gyro;
        m_telemetry = telemetry;

        m_absoluteHeading = m_gyro.getAbsoluteHeading();
        m_heading = m_gyro.getHeading();
        m_rotation2d = m_gyro.getRotation2d();
    }

    public double getAbsoluteHeading() {
        return m_absoluteHeading;
    }

    public double getHeading() {
        return m_heading;
    }

    public Rotation2d getRotation2d() { return m_rotation2d; }

    @Override
    public void periodic() {
        m_absoluteHeading = m_gyro.getAbsoluteHeading();
        m_heading = m_gyro.getHeading();
        m_rotation2d = m_gyro.getRotation2d();

       //  m_telemetry.addData("Heading", m_absoluteHeading );
        Log.w("IMU", "Heading=" +m_heading );

    }
}
