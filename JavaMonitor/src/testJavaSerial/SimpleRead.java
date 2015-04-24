package testJavaSerial;

import jssc.SerialPort;
import jssc.SerialPortEvent;
import jssc.SerialPortEventListener;
import jssc.SerialPortException;
 
public class SimpleRead {
 
    static SerialPort serialPort;
 
    public static void main(String[] args) {
        serialPort = new SerialPort("/dev/ttyUSB0"); 
        try {
            serialPort.openPort();
            serialPort.setParams(115200, 8, 1, 0);
            //Preparing a mask. In a mask, we need to specify the types of events that we want to track.
            //Well, for example, we need to know what came some data, thus in the mask must have the
            //following value: MASK_RXCHAR. If we, for example, still need to know about changes in states 
            //of lines CTS and DSR, the mask has to look like this: SerialPort.MASK_RXCHAR + SerialPort.MASK_CTS + SerialPort.MASK_DSR
            int mask = SerialPort.MASK_RXCHAR;
            //Set the prepared mask
            serialPort.setEventsMask(mask);
            //Add an interface through which we will receive information about events
            serialPort.addEventListener(new SerialPortReader());
            System.out.println("[Listener] Start");
        }
        catch (SerialPortException ex) {
            System.out.println(ex);
        }
    }
 
    static class SerialPortReader implements SerialPortEventListener {
 
        public void serialEvent(SerialPortEvent event) {
            //Object type SerialPortEvent carries information about which event occurred and a value.
            //For example, if the data came a method event.getEventValue() returns us the number of bytes in the input buffer.
            if(event.isRXCHAR()){
                if(event.getEventValue() > 0){
                    try {
                        byte buffer[] = serialPort.readBytes(1);
                        System.out.println("Received 1 bytes");
                        System.out.println(buffer[0]);
                    }
                    catch (SerialPortException ex) {
                        System.out.println(ex);
                    }
                }
            }
            //If the CTS line status has changed, then the method event.getEventValue() returns 1 if the line is ON and 0 if it is OFF.
            else if(event.isCTS()){
                if(event.getEventValue() == 1){
                    System.out.println("CTS - ON");
                }
                else {
                    System.out.println("CTS - OFF");
                }
            }
            else if(event.isDSR()){
                if(event.getEventValue() == 1){
                    System.out.println("DSR - ON");
                }
                else {
                    System.out.println("DSR - OFF");
                }
            }
        }
    }
}