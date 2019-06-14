package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class GPIO_Messager extends Thread {
    String currentCommand = "";
    String currentMsgData = "";
    boolean isMessageReady = false;
    boolean isProcessing = false;

    OpMode callingOpMode;
    public GPIO_Messager(OpMode _callingOpMode) {
        callingOpMode = _callingOpMode;

    }

    public void sendMessageGPIO(Integer commmand, Integer msgData) {
        isMessageReady = true;
        isProcessing = true;

        currentCommand = Integer.toBinaryString(commmand);
        currentMsgData = Integer.toBinaryString(msgData);


    }
    @Override
    public void run() {
        while(!Thread.currentThread().interrupted()) {
            if (isMessageReady) {
                //flip clock to true, short sleep
                for(int i = 0; i < 4; i++) {
                    //flip clock, read off next signal bit, repeat
                }
                for(int i = 0; i < 6; i++) {
                }
            }
            safeSleep(1);
        }
    }
    public void safeSleep(int t) {
        try {
            Thread.sleep(t);
        }
        catch(InterruptedException mInterruptedException) {
            System.out.println("exiting comms loop." + mInterruptedException);
            Thread.currentThread().interrupt();
        }
    }
}
