package org.firstinspires.ftc.teamcode;

import android.util.ArrayMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.ScheduleCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.RobotConfigConstants;
import org.firstinspires.ftc.teamcode.command.DefaultDifferentialDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DifferentialDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IMU;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RevIMUVertical;

import java.util.List;
import java.util.Map;

public class RobotContainer {

    public final IMU gyro;
    // Subsystems
//    private final Drivetrain m_driveSubsystem;
    public final DifferentialDriveSubsystem drivetrain;

 //   private final ColorSensorSubsystem m_colorSensors;

    // Controller
    private final GamepadEx m_gamepad1;

    public final ArmSubsystem arm;
    public final IntakeSubsystem intake;

    public final CarouselSubsystem carousel;

    // Gyro
    public RevIMUVertical m_gyro;

    public Map<String, Object> m_telemetryItems;

    private TelemetryPacket m_telemPacket;

    public RobotContainer(boolean bTeleOp, HardwareMap hardwareMap, Gamepad gamepad1) {

        m_telemPacket = new TelemetryPacket();

        m_telemetryItems = new ArrayMap<String, Object>();

        m_gyro = new RevIMUVertical(hardwareMap, RobotConfigConstants.ROBOT_CONFIG_IMU);
        m_gyro.init();

        gyro = new IMU(m_gyro, this);

        arm = new ArmSubsystem( hardwareMap, this );
        intake = new IntakeSubsystem( hardwareMap, this);

        carousel = new CarouselSubsystem( hardwareMap, this );
//        m_colorSensors = new ColorSensorSubsystem(  hardwareMap.get(NormalizedColorSensor.class, "cs1"), telemetry );

//        m_driveSubsystem = new Drivetrain(hardwareMap, telemetry, m_IMUSubsystem);
         drivetrain = new DifferentialDriveSubsystem(hardwareMap, this);

        // Enable bulk reads on hub
        // obtain a list of hubs
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        m_gamepad1 = new GamepadEx(gamepad1);

        if (bTeleOp) {
            configureButtonBindings();
        } else {
            // stop drivetrain if not being commanded otherwise during loop
            drivetrain.setDefaultCommand( new RunCommand( drivetrain::stop, drivetrain ) );
        }


    }

    public GamepadEx getGamepad1() {
        return m_gamepad1;
    }

    public ArmSubsystem getArmSubsystem() {
        return arm;
    }

    private void configureButtonBindings() {
     drivetrain.setDefaultCommand(
             new DefaultDifferentialDrive(drivetrain , () -> m_gamepad1.getLeftX(),
                     () -> m_gamepad1.getLeftY() / 2
                     )
     );

     // make sure carousel stops if we are not telling it to do anything else
     carousel.setDefaultCommand( new RunCommand( carousel::stop, carousel) );

/*        m_gamepad1.getGamepadButton(Button.DPAD_UP).whenPressed(new InstantCommand(
                () -> arm.setPower(.5)))
                .whenReleased(new InstantCommand(() -> arm.setPower(0)));

        m_gamepad1.getGamepadButton(Button.DPAD_DOWN).whenPressed(new InstantCommand(
                () -> arm.setPower(-.5)))
                .whenReleased(new InstantCommand(() -> arm.setPower(0)));
*/
        m_gamepad1.getGamepadButton(Button.A).whenPressed(new InstantCommand(
                () -> arm.goToPosition((int)Constants.ArmConstants.POSITION1)));

        m_gamepad1.getGamepadButton(Button.B).whenPressed(new InstantCommand(
                () -> arm.goToPosition((int)Constants.ArmConstants.POSITION2)));

        m_gamepad1.getGamepadButton(Button.Y).whenPressed(new InstantCommand(
                () -> arm.goToPosition((int)Constants.ArmConstants.POSITION3)));

        // turn off arm motor so falls to bottom position
        m_gamepad1.getGamepadButton(Button.X).whenPressed(new InstantCommand(
                () -> arm.setPower(0) ) );

        m_gamepad1.getGamepadButton(Button.LEFT_BUMPER).toggleWhenPressed(
                new InstantCommand(intake::suck),
                new InstantCommand(intake::stop)
        );

        m_gamepad1.getGamepadButton(Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(intake::eject)
        ).whenReleased(intake::stop);


        m_gamepad1.getGamepadButton(Button.DPAD_LEFT).whenHeld(
                new RunCommand(carousel::forward, carousel)
        );

        m_gamepad1.getGamepadButton(Button.DPAD_RIGHT).whenHeld(
                new RunCommand(carousel::backward, carousel)
        );


//                .whenReleased( new InstantCommand( () -> m_armSubsystem.setPower(0)));
/*
     m_gamepad1.getGamepadButton(Button.X).whenPressed( new SequentialCommandGroup(
             new DriveForward(m_driveSubsystem, 1).withTimeout(1000),
        new RotateToHeading(m_driveSubsystem, m_IMUSubsystem, 90, telemetry)
             ));

        // make button A drive forward for 4 seconds or until blue detected.
        m_gamepad1.getGamepadButton(Button.A).whenPressed(
            cc      new DriveForward(m_driveSubsystem, 1).withTimeout(4000)
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

    public void addTelem(String name, Object value) {
        m_telemetryItems.put(name, value);
    }

    public TelemetryPacket getTelemPacket() {
        return m_telemPacket;
    }

    public void sendTelem(FtcDashboard dashboard) {
        m_telemPacket.putAll( m_telemetryItems );
        dashboard.sendTelemetryPacket( m_telemPacket);
        m_telemPacket = new TelemetryPacket();

    }
}
