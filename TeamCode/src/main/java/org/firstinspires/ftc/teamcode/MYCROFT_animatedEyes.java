package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import java.util.LinkedList;

//TODO Restructure code to support a state machine
//TODO Test functionality in general
//TODO Decide what pair of phones to use and verify configuration
//TODO Add infrastrucure for mycroft-app controller communication
//TODO Animate the mycroft eyes themselves

@TeleOp (name="animated eyes TeleOp", group="TeleOp")
public class MYCROFT_animatedEyes extends OpMode {

    MYCROFT_RB robotBase;

    long shooterTime = 1000;
    long shooterStart = 0;

    byte currentDriveDist = 0;
    byte currentTurnAngle = 0;

    Toggle collector;
    boolean writingToQueue = true;
    long timeLast;
    LinkedList mycroftQueue;

    public DigitalChannel digChan, clockChan;
    GPIO_Monitor signal;
    byte currentMsg = 0;

    boolean manualReloadIsRunning = false;
    boolean manualReloadIsFinishing = false;
    private byte currentData;

    @Override
    public void init() {
        robotBase = new MYCROFT_RB();
        robotBase.init(hardwareMap, this);
        robotBase.setReloadAfterShot(true);

        signal = new GPIO_Monitor(this);

        collector = new Toggle();

        signal.start();

        digChan = hardwareMap.digitalChannel.get("digChan");
        clockChan = hardwareMap.digitalChannel.get("clockChan");
        digChan.setMode(DigitalChannel.Mode.INPUT);
        clockChan.setMode(DigitalChannel.Mode.INPUT);
        //digChan.setMode(DigitalChannel.Mode.OUTPUT);
        //digChan.setState(false);
        LinkedList mycroftQueue = new LinkedList();
        timeLast = System.currentTimeMillis();
        robotBase.startTurn(30, .5);
    }

    @Override
    public void loop() {
        if (timeLast - System.currentTimeMillis() >= 10 && writingToQueue) {
            timeLast = System.currentTimeMillis();
        }

        // doesn't allocate memory, creates pointer to byte array in signal
        byte[] msgAndData = signal.getMessage();
        if (msgAndData != null) {
            System.out.println("SSS msg " + msgAndData[0]);
            System.out.println("SSS data " + msgAndData[1]);

            //Drive straight command
            if(msgAndData[0] == 11) {
                robotBase.startBeeline(msgAndData[1], .5);
            }

            //Drive backwards command
            if(msgAndData[0] == 14) {
                robotBase.startBeeline(-msgAndData[1], .5);
            }

            //Shooter command
            if(msgAndData[0] == 3) {
                if(collector.update(true)) {
                    robotBase.startShooter();
                    shooterStart = System.currentTimeMillis();
                    System.out.println("SSS started shot");
                }
            }

            //Turn command
            if(msgAndData[0] == 6) {
                robotBase.startTurn(30, .5);
            }
        }

        if (System.currentTimeMillis() - shooterStart > shooterTime) {
            robotBase.stopShooter();
        }

        robotBase.beelineHandler();
        robotBase.turnHandler();
    }


    public byte getSignalState(){
        if(digChan.getState()) return 1;
        return 0;
    }
    public byte getClockState(){
        if(clockChan.getState()) return 1;
        return 0;
    }
}

