package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import androidx.annotation.NonNull;

public class MotorExDebug extends MotorEx {

    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     */
    public MotorExDebug(@NonNull HardwareMap hMap, String id) {
        super(hMap, id);
    }

    /**
     * Constructs the instance motor for the wrapper
     *
     * @param hMap        the hardware map from the OpMode
     * @param id          the device id from the RC config
     * @param gobildaType the type of gobilda 5202 series motor being used
     */
    public MotorExDebug(@NonNull HardwareMap hMap, String id, @NonNull GoBILDA gobildaType) {
        super(hMap, id, gobildaType);
    }

    /**
     * Constructs an instance motor for the wrapper
     *
     * @param hMap the hardware map from the OpMode
     * @param id   the device id from the RC config
     * @param cpr  the counts per revolution of the motor
     * @param rpm  the revolutions per minute of the motor
     */
    public MotorExDebug(@NonNull HardwareMap hMap, String id, double cpr, double rpm) {
        super(hMap, id, cpr, rpm);
    }

    @Override
    public void set(double output) {
        if (runmode == RunMode.VelocityControl) {
            double speed = bufferFraction * output * ACHIEVABLE_MAX_TICKS_PER_SECOND;
            double correctedVelo = getVelocity();
            double accel = getAcceleration();
            double veloCalc = veloController.calculate(correctedVelo, speed);
            double ffCalc = feedforward.calculate(speed, accel);
            double velocity = veloCalc + ffCalc;
 //           Log.w("MOTOREX", "SpeedTarget ticks/sec=" + speed + ", current vel = " + correctedVelo + ", current accel = " + accel + ", veloCalc=" + veloCalc + ", ffCalc=" + ffCalc + ", newVel=" + velocity + " (" + (velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND) + ")" );
            motorEx.setPower(velocity / ACHIEVABLE_MAX_TICKS_PER_SECOND);
        } else if (runmode == RunMode.PositionControl) {
            double error = positionController.calculate(encoder.getPosition());
            motorEx.setPower(output * error);
        } else {
            motorEx.setPower(output);
        }
    }
}
