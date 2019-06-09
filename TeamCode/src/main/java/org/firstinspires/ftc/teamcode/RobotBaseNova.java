package org.firstinspires.ftc.teamcode;

/**
 * Created by Lena on 6/2/2019.
 */

import android.hardware.Sensor;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


import static android.content.Context.SENSOR_SERVICE;

public class RobotBaseNova {

    public static double reloadResetTime = -1;

    float leftLiftPower;
    float rightLiftPower;

    int index;
    double dScale;

    //Hardware reference variables
    public DcMotor motorFrontLeft = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackRight = null;
    public DcMotor motorShooter = null;
    public DcMotor motorSpinner = null;
    public DcMotor motorLifterLeft = null;
    public DcMotor motorLifterRight = null;
    public DcMotor encoderMotor = null;
    public TouchSensor touchPow = null;
    public TouchSensor touchShooter = null;
    public Servo reloaderServo = null;
    public CRServo grabberServo = null;
    public Servo lifterLeftServo = null;
    public Servo lifterRightServo = null;

    //Allows the program to decide whether to set a default 'zero' orientation
    public boolean hasBeenZeroed= false;

    //Used to help time reloadHandler and shooterHandler
    double timeToFinishReload = -1;

    //hardware map, opmode, & vuforia
    HardwareMap hwMap = null;
    OpMode callingOpMode;
    VuforiaLocalizer vuforia;

    //sets positions for ball indexer (servo)
    static final double RELOADER_CLOSED = 0.32;
    static final double RELOADER_OPEN = 1;

    //more required vars for gyro operation
    private SensorManager mSensorManager;
    private Sensor mRotationVectorSensor;

    // This is relative to the initial position of the robot.
    // Possible values are:  0-360
    // 0 is set as straight ahead of the robot, 90 is the right, 270 is to the left
    public float zRotation;

    //helps with turn/push math to give an idea of where the beacon is/was
    public double lastPicBeaconAvg;

    //for reloadHandler and shooterHandler's usage
    boolean isReloadResetting = false;
    public boolean shooterIsBusy = false;
    public boolean touchToggle = false;
    static boolean reloadJustFinished = false;
    public boolean reloadAfterShot = false;

    //defines each possible amount of power we are able to give to the motor based on joystick
    private static final double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
            0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

    /**
     * Initializes Hardware map, motors, servo & sensors,
     * @param ahwMap to save reference to the Hardware map
     * @param _callingOpMode to save reference to the OpMode
     */
    public void init(HardwareMap ahwMap, OpMode _callingOpMode) {
        // Save reference to OpMode
        callingOpMode = _callingOpMode;
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorFrontLeft = hwMap.dcMotor.get("frontLeft");
        motorBackLeft = hwMap.dcMotor.get("backLeft");
        motorFrontRight = hwMap.dcMotor.get("frontRight");
        motorBackRight = hwMap.dcMotor.get("backRight");
        encoderMotor = hwMap.dcMotor.get("frontLeft");

        motorSpinner = hwMap.dcMotor.get("spinner");
        motorShooter = hwMap.dcMotor.get("shooter");

        motorLifterLeft = hwMap.dcMotor.get("lifterLeft");
        motorLifterRight = hwMap.dcMotor.get("lifterRight");

        motorLifterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLifterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Sets motors to drive in the correct directions
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        updateDriveMotors(0,0,false);

        // Set all motors to run without encoders.
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize servos.
        reloaderServo = hwMap.servo.get("reloader");
        grabberServo = hwMap.crservo.get("grabber");
        lifterLeftServo = hwMap.servo.get("left");
        lifterRightServo = hwMap.servo.get("right");
        grabberServo.setPower(0);
        lifterLeftServo.setPosition(0.2);
        lifterRightServo.setPosition(0);

        // Define and initialize touch sensors
        touchShooter = hwMap.touchSensor.get("touchShooter");
        touchPow = hwMap.touchSensor.get("touchPow");

        //resets all encoders
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //moves servo to preset position
        reloaderServo.setPosition(RELOADER_CLOSED);
    }

    /**
     * Handles the servo that is connected to the indexer and allows balls to be reloaded into the beacon, TeleOp compatible
     * @param reloadRequested only uses handler if reload is requested to make TeleOp compatible. If Tele, set equal to a button. If Auto, just put it in a while loop
     * @return to make Auto usable, put it in a loop, return true only while reloading
     */
    public boolean reloadHandler(boolean reloadRequested) {
        //If the time is more than the time it takes to reload
        if(callingOpMode.getRuntime() > timeToFinishReload) {
            reloadJustFinished = true;                                      //we know we just finished a reload
            if (teleOpDebug) {
                System.out.println("SSS finished reload");
            }
        }
        //If we want a reload, the reset time is 'null', and we're not resetting the reload
        if(reloadRequested && reloadResetTime == -1 && !isReloadResetting) {
            reloaderServo.setPosition(RELOADER_OPEN);                       //Open the reloader for a ball
            reloadResetTime = callingOpMode.getRuntime() + 0.6;             //Put reload reset time to current time + 0.4 for timing
            timeToFinishReload = callingOpMode.getRuntime() + 1.2;          //Change the time to finish reload for timing
            if (teleOpDebug) {
                System.out.println("SSS beginning reload");
            }
            return true;                                                    //Tell loop we're doing stuff
        }
        //Elseif we've been running for more time than it takes to reload and the reset time isn't 'null'
        else if(callingOpMode.getRuntime() > reloadResetTime && reloadResetTime != -1) {
            reloaderServo.setPosition(RELOADER_CLOSED);                     //Close the reloader to stop balls
            reloadResetTime = -1;                                           //Put the reset time back to 'null'
            isReloadResetting = true;                                       //tell the handler we are resetting the reloader now
            if (teleOpDebug) {
                System.out.println("SSS closing reload");
            }
            return true;                                                    //Tell loop we're doing stuff
        }
        //Elseif we've not been running long enough to get stuff done
        else if(callingOpMode.getRuntime() < timeToFinishReload) {
            if (teleOpDebug) {
                System.out.println("SSS still in reload");
            }
            return true;                                                    //Tell loop we're still working on doing stuff
        }
        //If none of the above are true, we aren't doing anything. Tell handler and loop so.
        isReloadResetting = false;
        return false;
    }

    /**
     * !! CURRENTLY NO LIFTER Used to run cap ball lifter and is TeleOp compatible with joysticks in for arguments, Auto compatible in a loop
     * @param leftPower to input amount of power for left lifter motor
     * @param rightPower to input amount of power for right lifter motor
     */
    public void lifterHandler(float leftPower, float rightPower){
        //If power is too big or small, put it at legal power
        leftLiftPower = Range.clip(rightPower, -1, 1);
        rightLiftPower = Range.clip(leftPower, -1, 1);

        //Set to one of 16 ok powers to scale
        rightLiftPower = (float)scaleInput(rightPower);
        leftLiftPower =  (float)scaleInput(leftPower);

        //Give the motors power
        motorLifterRight.setPower(rightLiftPower);
        motorLifterLeft.setPower(leftLiftPower);
    }

    /**
     * Sets drive motors with 100% legal powers, Tele compatible, used inside methods, DO NOT CALL FROM AUTO
     * @param left the power being input for the left side
     * @param right the power being input for the right side
     * @param slowDrive can turn on a slow-driving mode for TeleOp drivers
     */
    public void updateDriveMotors(double left, double right, boolean slowDrive){
        // tank drive
        // note that if y equal -1 then joystick is pushed all of the way forward.

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        //spin = Range.clip(spin, -1, 1); *reinstate if turned back to float

        //For slow drive mode, divide pwr for each side by e
        if(slowDrive){
            right /= 2.71828182845904523536028747135266249775724709369995957496696762772;
            left /= 2.71828182845904523536028747135266249775724709369995957496696762772;
        }
        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        right = scaleInput(right);
        left =  scaleInput(left);
        //spin =  (float)scaleInput(spin);

        // write the values to the drive motors
        motorFrontRight.setPower(right);
        motorBackRight.setPower(right);
        motorFrontLeft.setPower(left);
        motorBackLeft.setPower(left);
    }

    /**
     * Used to run particle shooter, Tele & Auto compatible. For tele set booleans to buttons, for Auto, set up in loop
     * @param shotRequested Only takes shot if requested to make Tele compatible.
     * @param manualRequested Only manually runs shoot motor if requested to make Tele compatible
     * @return whether the shooter is doing stuff to make Auto compatible with loops
     */
    static boolean teleOpDebug = false;
    static int loopCounter = 0;
    public boolean shooterHandler(boolean shotRequested, boolean manualRequested){
        if (teleOpDebug) {
            System.out.println("SSS in Shooter");
        }
        //case 0 - shoot isn't busy and nothing is requested
        if ((!shooterIsBusy && !manualRequested && !shotRequested) || callingOpMode.getRuntime() < timeToFinishReload){
            motorShooter.setPower(0);                                       //Confirm stuff is off, we're doing nothing
            shooterIsBusy = false;
            if(teleOpDebug) {
                System.out.println("SSS case 0");
            }
            return false;
        }

        // case 1 - shoot isn't busy and manual is requested
        else if (manualRequested){
            motorShooter.setPower(0.7);                                     //Turn on power, return we're doing nothing (no subroutines)
            shooterIsBusy = false;
            if (teleOpDebug) {
                System.out.println("SSS case 1");
            }
            return false;
        }

        // case 2 - shooter isn't busy and shot was requested
        else if (!shooterIsBusy && shotRequested){
            motorShooter.setPower(0.7);                                     //Turn on power
            shooterIsBusy = true;                                           //We're finally doing something
            touchToggle = false;                                            //Touch sensor hasn't been released
            if (teleOpDebug) {
                System.out.println("SSS case 2");
            }
            return true;                                                    //we're in a subroutine
        }

        // case 3 - shoot is busy and touch sensor not yet released (active)
        else if (shooterIsBusy && touchShooter.isPressed() && !touchToggle){
            if (teleOpDebug) {
                System.out.println("SSS case 3");
            }
            return true;                                                    //We're in a subroutine. Will keep motors on
        }

        // case 4 - shoot is still busy and touch sensor is released (not active)
        else if (shooterIsBusy && !touchShooter.isPressed()){
            touchToggle = true;                                             //Touch sensor has been released
            if (teleOpDebug) {
                System.out.println("SSS case 4");
            }
            return true;                                                    //We're in a subroutine still
        }

        // case 5 - shoot is busy and touch sensor has been released but is now active
        else if (shooterIsBusy && touchToggle && touchShooter.isPressed()){
            shooterIsBusy = false;                                          //We're not doing anything anymore
            motorShooter.setPower(0);                                       //Stop trying to move
            if (reloadAfterShot) reloadHandler(true);                       //If we want to reload, run the reloader
            if (teleOpDebug) {
                System.out.println("SSS case 5");
            }
            return false;                                                   //The subroutine is over
        }
        return false;                                                       //If none of above (never), we're doing nothing
    }

    /**
     * Scale an input to the scaleArray (for teleop driving)
     * @param dVal
     * @return
     */
    private double scaleInput(double dVal)  {

        // get the corresponding index for the scaleInput array.
        index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }

    /**
     * Set motor spinner to a particular power
     * @param power the power it's set to
     */
    public void setMotorSpinner(double power) {
        motorSpinner.setPower(power);
    }

    /**
     * Tell if the shooter is cocked
     * @return
     */
    public boolean isCocked(){
        return touchShooter.isPressed();
    }

    /**
     * Tell if the collector is running
     * @return
     */
    public boolean spinnerIsRunning() {
        if(motorSpinner.getPower() != 0) {
            return true;
        }
        else {
            return false;
        }
    }

    /**
     * safely tell whether we reload after our shots
     * @param _reload
     */
    public void setReloadAfterShot (boolean _reload) { reloadAfterShot = _reload; }



}