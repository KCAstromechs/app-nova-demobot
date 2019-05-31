package org.firstinspires.ftc.teamcode;

public class GPIO_Monitor extends Thread {
    boolean msgReady = false;
    byte msg = (byte)255;
    byte data = 0;

    public MYCROFT_animatedEyes callingOpMode;
    GPIO_Monitor(MYCROFT_animatedEyes _callingOpMode) {
        callingOpMode = _callingOpMode;
    }

    public byte[] getMessage() {
        if (msgReady) {
            //create new array and set vals
            byte[] arr = new byte[2];
            arr[0] = msg;
            arr[1] = data;

            msgReady = false;
            return arr;
        }
        else
            return null; //don't start another array if msg not ready
    }
    /*
    public byte[] getMsg(boolean clear) {
        if (msgReady) {
            if(clear) msgReady = false;
            return msg;
        }
        else {
            System.out.println("SSA:" + (byte) 127);
            return (byte) 127;
        }
    }
    */

    @Override
    public void run() {
        byte msg_buf;
        byte data_buf;
        byte lastClockState;
        int i;

        try {
            while(true) {
                if (callingOpMode.getClockState() == 1) {
                    lastClockState = 1;
                    msg_buf = 0;
                    System.out.println("SSS new loop");
                    data_buf = 0;
                    i = 0;
                    while(true) {
                        // Wait for clock signal to flip, telling us that the next bit is ready
                        if(callingOpMode.getClockState() == lastClockState) {
                            sleep(1);      // don't rev-up the CPU
                            continue;
                        }
                        lastClockState = (callingOpMode.getClockState());

                        // Read the bit and set inside the appropriate buffer
                        if(i>=4) {
                            data_buf += callingOpMode.getSignalState() << (i - 4);
                        }
                        else {
                            msg_buf += callingOpMode.getSignalState() << i;
                        }
                        //System.out.println("SSS" + callingOpMode.getSignalState());
                        i++;
                        if (i>=10) break;
                    }
                    msg = msg_buf;
                    data = data_buf;
                    msgReady = true;
                    System.out.println("broke out");
                    sleep(1000);
                    System.out.println("Done with message");
                }

                /*
                // Look for initial high signal
                if (callingOpMode.getGPIOstate() == 0) {
                    sleep(50);
                    continue;
                }
                sleep(msPerBit);

                // Read message bits (one per 10ms)
                msg_buf = 0;
                for (int i = 0; i < 4; i++) {
                    msg_buf += callingOpMode.getGPIOstate() << i;
                    sleep(msPerBit);
                }

                // Verify 500ms of quiet time
                x = 0;
                while (x < 5 && callingOpMode.getGPIOstate() == 0) {
                    sleep(msPerBit);
                    x++;
                }
                if(x != 5) continue;

                // Queue up message
                msg = msg_buf;
                msgReady = true;
            }
            */
            }
        } catch (Exception e) {
            System.out.print("SSS" + e);
        }

    }
}
