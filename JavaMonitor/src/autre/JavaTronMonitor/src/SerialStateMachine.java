
public class SerialStateMachine {
	public enum State{
		Idle,
		Reading,
		Escape,
		Error
	}
	
	public State state;
	public TronFrame fenetre;
	
	byte START_BYTE = 18;
	byte STOP_BYTE = 19;
	byte ESCAPE_BYTE = 125;
	
	byte[] msgBuffer;
	int msgSize;
	
	public int idAddHeadMsg;
	
	public SerialStateMachine(TronFrame fenetre, int idAddHeadMsg) {
		this.state = State.Idle;
		this.idAddHeadMsg = idAddHeadMsg;
		this.fenetre = fenetre;
		msgBuffer = new byte[16];
		msgSize = 0;
	}
	
	public void readMsg(int size) {
		System.out.println("readMsg");
		if(size == 7) {
			if(msgBuffer[1] == idAddHeadMsg) {
				System.out.println("addHead x: "+msgBuffer[4]+", y: "+msgBuffer[5]+", id: "+msgBuffer[6]);
				this.fenetre.paintSquare((int) msgBuffer[4], (int) msgBuffer[5], (int) msgBuffer[6]);
			}
		}
	}
	
	public void receiveByte(byte b) {
		switch(this.state) {
		case Idle:
			if(b == START_BYTE) {
				this.state = State.Reading;
			}
			break;
			
		case Reading:
			if(b == STOP_BYTE) {
				readMsg(msgSize);
				msgSize = 0;
				this.state = State.Idle;
			} else {
				if(b == ESCAPE_BYTE) {
					this.state = State.Escape;
				} else {
					if(msgSize < 16) {
						msgBuffer[msgSize] = b;
						msgSize++;
					} else {
						this.state = State.Error;
					}
				}
			}
			break;
			
		case Escape:
			if(msgSize < 16) {
				msgBuffer[msgSize] = b;
				msgSize++;
				this.state = State.Reading;
			} else {
				this.state = State.Error;
			}
			break;
			
		case Error:
			System.out.println("[Serial] Error: buffer overflow");
			this.state = State.Idle;
			msgSize = 0;
			break;
		}
	}
}
