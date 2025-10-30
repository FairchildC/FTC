package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp
public class RobotTeleOp extends OpMode {

    MecanumDrive drive = new MecanumDrive();
    Launcher launcher = new Launcher();

    @Override
    public void init() {
        drive.init(hardwareMap);
        launcher.init(hardwareMap);

    }

    @Override
    public void loop() {

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        // Note: pushing left stick forward gives negative value
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // The user has control of the speed of launcher motor without automatically
        // queuing a shot.
        if (gamepad1.y) {
            launcher.startLauncher();
        }
        else if (gamepad1.b) {
            launcher.stopLauncher();
        }

        // update launcher state machine
        launcher.updateState();

        telemetry.addData("State", launcher.getState());
        telemetry.addData("Launcher Velocity", launcher.getVelocity());
    }
}


