
import javax.swing.SwingUtilities;
import jssc.SerialPort;
import jssc.SerialPortEvent;
import jssc.SerialPortEventListener;
import jssc.SerialPortException;

public class TronMonitor {
	public static TronFrame fenetre;
	static SerialPort serialPort;
	static byte[] tronGoMsg;
	
	public static SerialStateMachine machine;
	
	public static void tronGoInit(byte x, byte y, byte id) {
		int msgSize = 9;
		if(x == 19) {msgSize++;}
		if(y == 19) {msgSize++;}
		tronGoMsg = new byte[msgSize];
		
		tronGoMsg[0] = 18;
		tronGoMsg[1] = 0;
		tronGoMsg[2] = 6;
		tronGoMsg[3] = 0;
		tronGoMsg[4] = 3;
		
		int t = 5;
		if(x == 19) {
			tronGoMsg[t] = 125;
			t++;
		}
		tronGoMsg[t] = x;
		t++;
		if(y == 19) {
			tronGoMsg[t] = 125;
			t++;
		}
		tronGoMsg[t] = y;
		t++;
		tronGoMsg[t] = id;
		t++;
		tronGoMsg[t] = 19;
		
	}    
	public static void main(String[] args){
		
		fenetre = new TronFrame();
		fenetre.setVisible(true);
		machine = new SerialStateMachine(fenetre, 6);
		
		
		//fenetre.paintSquare(0, 0, 0);
		serialPort = new SerialPort("/dev/ttyUSB0"); 
		//serialPort = new SerialPort("/dev/ttyACM0"); 
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
            serialPort.addEventListener(new SerialPortReader(machine));
            
            
            
            System.out.println("[Listener] Start");
           
            //Test -------------------------------------
            System.out.println("[Listener] Sending: ");
            for(byte i = 0; i < 20; i++) {
                tronGoInit(i, i, (byte) 0);
                serialPort.writeBytes(tronGoMsg);
                tronGoInit((byte) (i+1), i, (byte) 0);
                serialPort.writeBytes(tronGoMsg);
            }
            //Fin test ----------------------------------
            
        }
        catch (SerialPortException ex) {
            System.out.println(ex);
        }
		
	}
	
	
	static class SerialPortReader implements SerialPortEventListener {
		 
        private SerialStateMachine machine;
        
        public SerialPortReader(SerialStateMachine machine) {
        	super();
        	this.machine = machine;
        }
        

		public void serialEvent(SerialPortEvent event) {
            //Object type SerialPortEvent carries information about which event occurred and a value.
            //For example, if the data came a method event.getEventValue() returns us the number of bytes in the input buffer.
            if(event.isRXCHAR()){
                if(event.getEventValue() > 0){
                    try {
                        byte buffer[] = serialPort.readBytes(1);
                        System.out.print("Received: ");
                        System.out.println(buffer[0]);
                        this.machine.receiveByte(buffer[0]);
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