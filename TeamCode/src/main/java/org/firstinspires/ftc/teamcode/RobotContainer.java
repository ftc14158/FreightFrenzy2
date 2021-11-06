package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.RobotConfigConstants;
import org.firstinspires.ftc.teamcode.command.DefaultDrive;
import org.firstinspires.ftc.teamcode.command.DriveForward;
import org.firstinspires.ftc.teamcode.command.RotateToHeading;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.IMU;

import java.util.List;
import java.util.function.BooleanSupplier;

public class RobotContainer {

    private final IMU m_IMUSubsystem;
    // Subsystems
    private final Drivetrain m_driveSubsystem;

    private final ColorSensorSubsystem m_colorSensors;

    // Controller
    private final GamepadEx m_gamepad1;

    // Gyro
    public GyroEx m_gyro;

    public RobotContainer(HardwareMap hardwareMap, Gamepad gamepad1, Telemetry telemetry) {

        m_gyro = new RevIMU(hardwareMap, RobotConfigConstants.ROBOT_CONFIG_IMU);
        m_gyro.init();

        m_IMUSubsystem = new IMU(m_gyro, telemetry);

        m_colorSensors = new ColorSensorSubsystem(  hardwareMap.get(NormalizedColorSensor.class, "cs1"), telemetry );

        m_driveSubsystem = new Drivetrain(hardwareMap, telemetry, m_IMUSubsystem);

        // Enable bulk reads on hub
        // obtain a list of hubs
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        m_gamepad1 = new GamepadEx(gamepad1);

        configureButtonBindings(telemetry);
    }

    private void configureButtonBindings(Telemetry telemetry) {
     m_driveSubsystem.setDefaultCommand(
             new DefaultDrive( m_driveSubsystem, () -> m_gamepad1.getLeftY(),
                     () -> m_gamepad1.getRightX(), () -> m_gamepad1.getLeftX(),
                     () -> m_gamepad1.isDown( Button.LEFT_BUMPER),
                     () -> m_gamepad1.isDown( Button.RIGHT_BUMPER)
                     )
     );

     m_gamepad1.getGamepadButton(Button.Y).whenPressed( new InstantCommand(
         m_driveSubsystem::resetMotors, m_driveSubsystem));

     m_gamepad1.getGamepadButton(Button.X).whenPressed( new SequentialCommandGroup(
             new DriveForward(m_driveSubsystem, 1).withTimeout(1000),
        new RotateToHeading(m_driveSubsystem, m_IMUSubsystem, 90, telemetry)
             ));

        // make button A drive forward for 4 seconds or until blue detected.
        m_gamepad1.getGamepadButton(Button.A).whenPressed(
                new DriveForward(m_driveSubsystem, 1).withTimeout(4000)
                .interruptOn( m_colorSensors::isBlue )
        );



        m_gamepad1.getGamepadButton(Button.B).toggleWhenPressed( new RotateToHeading(m_driveSubsystem, m_IMUSubsystem, 90, telemetry));

     // Run each motor in turn when X button is pressed on keypad

     /*
     m_gamepad1.getGamepadButton(Button.X).whenPressed( new SequentialCommandGroup(

             new RunCommand( () -> m_driveSubsystem.setMotor(0, .5), m_driveSubsystem ).withTimeout(2000),
             new RunCommand( () -> m_driveSubsystem.setMotor( 0, 0), m_driveSubsystem ).withTimeout(1000),
        new RunCommand( () -> m_driveSubsystem.setMotor(1, .5), m_driveSubsystem ).withTimeout(2000),
                new RunCommand( () -> m_driveSubsystem.setMotor( 1, 0), m_driveSubsystem ).withTimeout(1000),
        new RunCommand( () -> m_driveSubsystem.setMotor(2, .5), m_driveSubsystem ).withTimeout(2000),
                new RunCommand( () -> m_driveSubsystem.setMotor( 2, 0), m_driveSubsystem ).withTimeout(1000),
        new RunCommand( () -> m_driveSubsystem.setMotor(3, .5), m_driveSubsystem ).withTimeout(2000),
                new InstantCommand( () -> m_driveSubsystem.setMotor( 3, 0), m_driveSubsystem )
     ) );
*/
    }
}
