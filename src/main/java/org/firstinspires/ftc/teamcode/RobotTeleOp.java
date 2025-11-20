package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

@TeleOp
public class RobotTeleOp extends OpMode {

    MecanumDrive drive = new MecanumDrive();
    Launcher launcher = new Launcher();
    Intake intake = new Intake();

    @Override
    public void init() {
        drive.init(hardwareMap);
        launcher.init(hardwareMap);
        intake.init(hardwareMap);
    }

    @Override
    public void loop() {

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        // Note: pushing left stick forward gives negative value
        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // The user has control of the speed of launcher motor without automatically
        // queuing a shot.
        if (gamepad2.y) {
            launcher.startLauncher();
        }
        else if (gamepad2.b) {
            launcher.stopLauncher();
        }
        //For Intake (test if same buttons works)
        if (gamepad2.a) {
            intake.startIntake();
        } else if (gamepad2.x) {
            intake.stopIntake();
        }

        //if (gamepad2.right_trigger > 0) {
        //    launcher.loadBall();
        //}

        // update launcher state machine
        launcher.updateState();

        telemetry.addData("State", launcher.getState());
        telemetry.addData("Upper Launch Velocity", launcher.getUpperVelocity());
        telemetry.addData("Lower Launch Velocity", launcher.getLowerVelocity());
    }
}


