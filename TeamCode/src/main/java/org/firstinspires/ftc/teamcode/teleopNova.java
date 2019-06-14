package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="Nova Teleop", group="Teleop")
public class teleopNova extends OpMode {

    RobotBaseNova robotBase;

    boolean released = true;
    boolean bReleased = false;
    boolean shotTaken = false;
    boolean reloading = false;
    boolean shotSequence = false;

    boolean manualReloadIsRunning = false;
    boolean manualReloadIsFinishing = false;

    public DigitalChannel digChan, clockChan;
    GPIO_Messager messagerThread;

    @Override
    public void init(){
        //digChan = hardwareMap.digitalChannel.get("digChan");
        //clockChan = hardwareMap.digitalChannel.get("clockChan");

        //messagerThread.start();

        robotBase = new RobotBaseNova();
        robotBase.init(hardwareMap, this);
        robotBase.setReloadAfterShot(true);
        RobotBaseNova.teleOpDebug = false;
    }

    @Override
    public void loop() {
        //Starts automatic shot sequence on either gamepad's left bumper,
        // and manually adjusts position of shooter on either gamepad's left trigger
        robotBase.shooterHandler(gamepad1.left_bumper, gamepad1.left_trigger > 0.2);

        //manually reloads a ball when gamepad1's right bumper is activated
        robotBase.reloadHandler(gamepad1.x);

        //Case 1: Button "B" is pressed, so run spinner in reverse
        if (gamepad1.b) {
            robotBase.setMotorSpinner(-1.0);
            bReleased = true;
        }

        //Case 2: Button "B" not pressed, but pressed last time through loop, so turn spinner off
        else if (bReleased) {
            robotBase.setMotorSpinner(0);
            bReleased = false;
        }
        //As long as the spinner is off...
        else if (!robotBase.spinnerIsRunning()) {
            //Case 3: A is bring pressed, either has not been pressed or has been recently released, so run spinner forward
            if ((gamepad1.a) && released) {
                robotBase.setMotorSpinner(1.0);
                released = false;
            }
            //Case 4: A is not being pressed, so allow to enter Cases 3&5 again
            else if (!(gamepad1.a)) {
                released = true;
            }
        }
        //As long as the spinner is on...
        else {
            //Case 5: A is being pressed, either has not been pressed or has been recently released, so turn the spinner off
            if ((gamepad1.a) && released) {
                robotBase.setMotorSpinner(0);
                released = false;
            }
            //Case 6: A is not being pressed, so allow to enter Cases 3&5 again
            else if (!(gamepad1.a)) {
                released = true;
            }
        }

        if(gamepad1.dpad_up){
            robotBase.motorLifterLeft.setPower(-0.75);
            robotBase.motorLifterRight.setPower(0.75);
        } else if (gamepad1.dpad_down){
            robotBase.motorLifterLeft.setPower(0.75);
            robotBase.motorLifterRight.setPower(-0.75);
        } else {
            robotBase.motorLifterLeft.setPower(0);
            robotBase.motorLifterRight.setPower(0);
        }

        //Set power based on the joysticks, set 'slow mode' based on right bumper
        robotBase.updateDriveMotors(-gamepad1.left_stick_y, -gamepad1.right_stick_y, gamepad1.right_bumper);
    }
    /*public void setSignalState(boolean state){
        digChan.setState(state);
    }
    public void setClockState(boolean state){
        clockChan.setState(state);
    }*/
}
