package org.firstinspires.ftc.teamcode;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.DrivetrainTracker;

import static android.content.Context.SENSOR_SERVICE;

/**
 * Created by N2Class1 on 8/26/2018.
 */

public class MYCROFT_RB implements SensorEventListener{

    static final double COUNTS_PER_MOTOR_REV = 1100;    // NeveRest Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;    // For figuring circumference
    static final double P_BEELINE_COEFF = 0.04;           // Larger is more responsive, but also less stable
    static final double P_TURN_COEFF = 0.018;          // Larger is more responsive, but also less stable
    static final double TURN_MINIMUM_SPEED = 0.4;       //Minimum speed turn can drive at during PID control
    static final int ANGLE_RESOLUTION = 6;              //Minimum # of degrees mycroft turn functionality can process.
    //encoder ticks per one inch
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    float leftLiftPower;
    float rightLiftPower;

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

    //Used to help time reloadHandler and shooterHandler
    double timeToFinishReload = -1;

    //hardware map, opmode, & vuforia
    HardwareMap hwMap = null;
    OpMode callingOpMode;

    //sets positions for ball indexer (servo)
    static final double RELOADER_CLOSED = 0.32;
    static final double RELOADER_OPEN = 1;

    //beeline loop method vars
    int target, currentPos;
    double power, leftPower, rightPower, error, correction;

    //for reloadHandler and shooterHandler's usage
    boolean isReloadResetting = false;
    public boolean shooterIsBusy = false;
    public boolean touchToggle = false;
    static boolean reloadJustFinished = false;
    public boolean reloadAfterShot = false;
    public static double reloadResetTime = -1;

    private DrivetrainTracker driveTrainGoal;

    private SensorManager mSensorManager;
    private Sensor mRotationVectorSensor;

    // This is relative to the initial position of the robot.
    // Possible values are:  0-360
    // 0 is set as straight ahead of the robot, 90 is the right, 270 is to the left
    public float zRotation;

    float[] rotationMatrix = new float[9];
    float[] orientation = new float[3];
    float zero, rawGyro;
    boolean hasBeenZeroed = false;

    //defines each possible amount of power we are able to give to the motor based on joystick
    private static final double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
            0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};

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

        setDrivetrainGoal(DrivetrainTracker.STOPPED, 0, 0);

        motorLifterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLifterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Sets motors to drive in the correct directions
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorLifterLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        updateDriveMotors(0,0);

        // Define and initialize servos.
        reloaderServo = hwMap.servo.get("reloader");

        // Define and initialize touch sensors
        touchShooter = hwMap.touchSensor.get("touchShooter");
        touchPow = hwMap.touchSensor.get("touchPow");

        //Accessing gyro and accelerometer from Android
        mSensorManager = (SensorManager) hwMap.appContext.getSystemService(SENSOR_SERVICE);
        mRotationVectorSensor = mSensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);
        mSensorManager.registerListener(this, mRotationVectorSensor, 10000);

        //resets all encoders
        resetEncoders();

        //moves servo to preset position
        reloaderServo.setPosition(RELOADER_CLOSED);
    }
    public void resetEncoders() {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void updateDriveMotors(double left, double right){
        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);


        // write the values to the drive motors
        motorFrontRight.setPower(right);
        motorBackRight.setPower(right);
        motorFrontLeft.setPower(left);
        motorBackLeft.setPower(left);
    }
    public void setDrivetrainGoal(DrivetrainTracker stat, double left, double right) {
        driveTrainGoal = stat;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        // write the values to the drive motors
        motorFrontRight.setPower(right);
        motorBackRight.setPower(right);
        motorFrontLeft.setPower(left);
        motorBackLeft.setPower(left);
    }
    double driveHeading = 0;
    double drivePower = 0;
    public void startBeeline(double inches, double power) {
        target = Math.abs((int) (inches * COUNTS_PER_INCH));
        resetEncoders();
        if(inches > 0) {
            drivePower = power;
            setDrivetrainGoal(DrivetrainTracker.FORWARD, drivePower, drivePower);
        }
        else {
            drivePower = -power;
            setDrivetrainGoal(DrivetrainTracker.REVERSE, drivePower, drivePower);
        }
    }
    //WIP untested
    public void beelineHandler(){
        if(!(driveTrainGoal == DrivetrainTracker.FORWARD || driveTrainGoal == DrivetrainTracker.REVERSE)) return;
        currentPos = Math.abs(motorFrontRight.getCurrentPosition());
        //Case 2: The drivetrain is going, but we haven't reached our goal yet

        if (currentPos < target){
            //keep driving

            error = driveHeading - zRotation;
            correction = Range.clip(error * P_BEELINE_COEFF, -1, 1);
            updateDriveMotors(drivePower, drivePower);
        }

        //Case 3: The drivetrain is going, but we hit our goal
        else {
            //stop to drive
            setDrivetrainGoal(DrivetrainTracker.STOPPED, 0, 0);
            resetEncoders();

        }
        return;
    }
    public void beeline(double inches, int heading) {
        int target = (int) (inches * COUNTS_PER_INCH);          //translates the number of inches to be driven into encoder ticks
        double power =1;

        double leftPower, rightPower, error, correction;

        //Ensure that motors are set up correctly to drive
        resetEncoders();



        //Find initial number of ticks on encoder when beginning drive
        double encoderInitialPos = encoderMotor.getCurrentPosition();
        callingOpMode.telemetry.addData("Initial position", encoderInitialPos);

        //While: we have not driven correct distance & bot is not stopped
        while (Math.abs(encoderMotor.getCurrentPosition()) < Math.abs(target) && !Thread.interrupted()) {
            //Put the power on and hit pause for a second
            error = heading - zRotation;
            correction = Range.clip(error * P_BEELINE_COEFF, -1, 1);
            leftPower = power + correction;
            rightPower = power - correction;
            leftPower = Range.clip(leftPower, -1.0, 1.0);
            rightPower = Range.clip(rightPower, -1.0, 1.0);
            updateDriveMotors(leftPower, rightPower);
            Thread.yield();
        }

        //When the drive is finished, it is time to turn off the drive motors
        updateDriveMotors(0, 0);

        //Reset the motors for future use, just in case
        resetEncoders();

    }

    int turnHeading = 0;
    double turnPower = 0;
    public void startTurn(int headingIndex, double power) {
        turnHeading = headingIndex * ANGLE_RESOLUTION;
        turnPower = power;
        target = (int) normalize360(zRotation + turnHeading);
        if(turnHeading > 0) {
            setDrivetrainGoal(DrivetrainTracker.CW_TURN, power, -power);
        }
        else {
            setDrivetrainGoal(DrivetrainTracker.CCW_TURN, -power, power);
        }
    }
    public void turnHandler(){
        boolean isTurning = (driveTrainGoal == DrivetrainTracker.CCW_TURN || driveTrainGoal == DrivetrainTracker.CW_TURN);
        if (!isTurning) return;
        else {
            if(Math.abs(zRotation - target) < 10) {
                // We've reached the target angle
                setDrivetrainGoal(DrivetrainTracker.STOPPED, 0,0);
            }
        }
        //Case 2: Not turning, and movement requested

    }

    public void turn(float turnHeading, double power) throws InterruptedException {
        int wrapFix = 0;                                        //Can be used to modify values and make math around 0 easier
        double rightPower;                                      //Power to be given to right side of bot
        double leftPower;                                       //Power to be given to left side of bot
        double motorSpeed = power;                              //Used to safely modify power by the deceleration at end of turn
        float shiftedTurnHeading = turnHeading;                 //Can be used in conjunction with wrapFix to make math around 0 easier
        //If heading is not on correct scale, put it between 0-360
        turnHeading = normalize360(turnHeading);
        //Figure out how far the robot would have to turn in counterclockwise & clockwise directions
        float cclockwise = zRotation - turnHeading;
        float clockwise = turnHeading - zRotation;

        //Normalize cwise & ccwise values to between 0=360
        clockwise = normalize360(clockwise);
        cclockwise = normalize360(cclockwise);

        int error = 1;                                          //sets the distance to the target gyro value that we will accept
        if (turnHeading - error < 0|| turnHeading + error > 360) {
            wrapFix = 180;                                      //if within the range where the clockmath breaks, shift to an easier position
            shiftedTurnHeading = normalize360(turnHeading + wrapFix);
        }

        //If it would be longer to take the ccwise path, we go *** CLOCKWISE ***
        if(Math.abs(cclockwise) >= Math.abs(clockwise)){
            //While we're not within our error, and we haven't overshot, and the bot is running
            while(Math.abs(normalize360(zRotation + wrapFix)- shiftedTurnHeading) > error &&
                    Math.abs(cclockwise) >= Math.abs(clockwise) && !Thread.interrupted()) {

                //Wait a hot decisecond
                Thread.sleep(10);

                //Take motor speed from clockwise, P coeff
                motorSpeed = clockwise*P_TURN_COEFF;

                //If necessary, pare down motorspeed, or bring it up to stall speed
                if(motorSpeed>power){
                    motorSpeed=power;
                } else if(motorSpeed < TURN_MINIMUM_SPEED){
                    motorSpeed = TURN_MINIMUM_SPEED;
                }

                //Set motor powers equal to motorSpeed
                leftPower=motorSpeed;
                rightPower=-motorSpeed;

                //Start driving
                updateDriveMotors(leftPower, rightPower);

                //Update lastError, lastTime, cwise & ccwise
                cclockwise = normalize360(zRotation - turnHeading);
                clockwise = normalize360(turnHeading - zRotation);
            }

        }
        //If it would take longer to take the cwise path, we go *** COUNTERCLOCKWISE ***
        else if(Math.abs(clockwise) > Math.abs(cclockwise)){
            //While we're not within our error, and we haven't overshot, and the bot is running
            while(Math.abs(normalize360(zRotation + wrapFix)- shiftedTurnHeading) > error &&
                    Math.abs(clockwise) > Math.abs(cclockwise) && !Thread.interrupted()) {

                //Stop a hot decisecond
                Thread.sleep(10);

                //Take motorSpeed from ccwise, P coeff
                motorSpeed = cclockwise*P_TURN_COEFF;

                //If necessary, pare down motorSpeed or bring it up to stall speed
                if(motorSpeed>power){
                    motorSpeed=power;
                }else if(motorSpeed < TURN_MINIMUM_SPEED){
                    motorSpeed = TURN_MINIMUM_SPEED;
                }

                //Set motor powers equal to motorSpeed
                leftPower=-motorSpeed;
                rightPower=motorSpeed;

                //Start driving
                updateDriveMotors(leftPower, rightPower);

                //Update lastError, lastTime, ccwise & cwise
                cclockwise = normalize360(zRotation - turnHeading);
                clockwise = normalize360(turnHeading - zRotation);

            }
        }

        //When all's said and done, stop driving
        updateDriveMotors(0, 0);
    }

    private double scaleInput(double dVal)  {
        int index;
        double dScale;
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
    public void stopMotors() {
        motorBackRight.setPower(0);
        motorBackLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorLifterLeft.setPower(0);
        motorLifterRight.setPower(0);
        motorShooter.setPower(0);
        motorSpinner.setPower(0);
    }

    /**
     *
     *
     *
     *
     * SUBSYSTEMS
     *
     *
     *
     *
     */

    public void startShooter() {
        motorShooter.setPower(1);
    }
    public void stopShooter() {
        motorShooter.setPower(0);
    }

    boolean teleOpDebug = false;
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

    public void setReloadAfterShot (boolean _reload) { reloadAfterShot = _reload; }

    public void setMotorSpinner(double power) {
        motorSpinner.setPower(power);
    }

    public boolean spinnerIsRunning() {
        if(motorSpinner.getPower() != 0) {
            return true;
        }
        else {
            return false;
        }
    }

    /**
     * When the gyro changes, jump down here to fix it
     * @param sensorEvent
     */
    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        SensorManager.getRotationMatrixFromVector(rotationMatrix, sensorEvent.values);
        SensorManager.getOrientation(rotationMatrix, orientation);

        rawGyro = (float) Math.toDegrees(orientation[0]);

        //If the zero hasn't been zeroed do the zero
        if (!hasBeenZeroed) {
            hasBeenZeroed = true;
            zero = rawGyro;
        }
        //Normalize zRotation to be used
        zRotation = normalize360(rawGyro - zero);
//        Dbg("zRotation in callback: " , zRotation, false);
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }

    /**
     * If 0-360 variables are not on a 0-360 scale, move them up and down 360 at a time to put them on one
     * @param val the variable to be changed
     * @return
     */
    public float normalize360(float val) {
        while (val > 360 || val < 0) {

            if (val > 360) {
                val -= 360;
            }

            if (val < 0) {
                val += 360;
            }
        }
        return val;
    }

}
