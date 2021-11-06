package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;

public class ColorSensorSubsystem extends SubsystemBase {

    /**
     * The colorSensor field will contain a reference to our color sensor hardware object
     */
    private NormalizedColorSensor colorSensor;

    private Telemetry m_telemetry;

    public ColorSensorSubsystem(NormalizedColorSensor colorSensor, Telemetry telemetry) {
        this.colorSensor = colorSensor;
        this.m_telemetry = telemetry;

        colorSensor.setGain(9);   // We tested and found gain 9 gave good readings

    }


    @Override
    public void periodic() {
        m_telemetry.addData("Is Red", isRed() ? "YES" : "NO" );
        m_telemetry.addData("Is Blue", isBlue() ? "YES" : "NO" );
    }

    /*
      Color sensor is detecting red if Red level is at least twice as much as Blue
      and Green is less then 75% of Red
     */
    public boolean isRed() {

        NormalizedRGBA currentColor;

        currentColor = colorSensor.getNormalizedColors();

        return (currentColor.red > (currentColor.blue * 2))
                && (currentColor.green < (currentColor.red * 0.75));
    }

    /*
      Color sensor is detecting blue if Blue level is at least twice as much as Red
      and Green is less then 75% of Blue
     */
    public boolean isBlue() {

        NormalizedRGBA currentColor;

        currentColor = colorSensor.getNormalizedColors();

        return (currentColor.blue > (currentColor.red * 2))
                && (currentColor.green < (currentColor.blue * 0.75));
    }


}
