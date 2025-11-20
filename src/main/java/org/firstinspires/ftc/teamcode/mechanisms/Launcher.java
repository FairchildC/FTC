package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Launcher {
    private final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    private final double START_POSITION = 0.0; //We send this power to the servos when we want them to stop.
    private final double FEED_POSITION = 0.25;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;
    final double LAUNCHER_STOP = 0;

    private DcMotorEx lowerLaunch, upperLaunch;
    private Servo launchFeeder;

    ElapsedTime feederTimer = new ElapsedTime();

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    public void init (HardwareMap hwMap) {
        upperLaunch = hwMap.get(DcMotorEx.class, "upper_launch");
        lowerLaunch = hwMap.get(DcMotorEx.class, "lower_launch");
        launchFeeder = hwMap.get(Servo.class,"launch_feeder");

        // Set launcher motor to RUN_USING_ENCODER and BRAKE to slow down faster than coasting.
        upperLaunch.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lowerLaunch.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        upperLaunch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lowerLaunch.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // add these lines when encoders have been attached to the launch motors
        upperLaunch.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                300, 0, 0, 10));
        lowerLaunch.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(
                300, 0, 0, 10));

        // Set left feeder servo to reverse so both servos work to feed ball into robot.
        launchFeeder.setDirection(Servo.Direction.REVERSE);

        // Set initial state of launcher to IDLE.
        launchState = LaunchState.IDLE;
        stopFeeder();
        stopLauncher();
    }

    public void stopFeeder() {
        // Set feeders to a preset value to stop the servos.
        launchFeeder.setPosition(START_POSITION);
    }

    public void updateState () {
        switch (launchState) {
            case IDLE:
                break;
            case SPIN_UP:
                upperLaunch.setVelocity(LAUNCHER_TARGET_VELOCITY);
                lowerLaunch.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if ((upperLaunch.getVelocity() > LAUNCHER_MIN_VELOCITY) &&
                        (lowerLaunch.getVelocity() > LAUNCHER_MIN_VELOCITY)){
                    // transition states
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                launchFeeder.setPosition(FEED_POSITION);
                feederTimer.reset();
                // transition state
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    stopFeeder();
                    // transition state
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }

    public void startLauncher(){
        if (launchState == LaunchState.IDLE) {
            // transition states
            launchState = LaunchState.SPIN_UP;
        }
    }

    public void stopLauncher () {
        stopFeeder();
        upperLaunch.setVelocity(LAUNCHER_STOP);
        lowerLaunch.setVelocity(LAUNCHER_STOP);
        launchState = LaunchState.IDLE;
    }

    public String getState() {
        return launchState.toString();
    }

    public double getUpperVelocity() {
        return upperLaunch.getVelocity();
    }

    public double getLowerVelocity() {
        return lowerLaunch.getVelocity();
    }


}
