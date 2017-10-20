import lejos.nxt.*;
import lejos.nxt.comm.*;
import java.io.*;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.util.Delay;
//import lejos.nxt.UltrasonicSensor;

public class SlaveSonar {
	private static final byte TRAVEL = 0; 
	private static final byte ROTATE = 1;
	private static final byte FULL_SCAN = 2; 
	private static final byte SINGLE_SCAN = 3; 
	private static final byte EXIT = 4; 

	static UltrasonicSensor sonar;
	static DataOutputStream dos;

	public static void main(String[] args) throws Exception {
		USBConnection btc = USB.waitForConnection(); /* USB communication */
		/* Uncomment next line for Bluetooth */
		//BTConnection btc = Bluetooth.waitForConnection();
		DataInputStream dis = btc.openDataInputStream();
		DataOutputStream dos = btc.openDataOutputStream();
		// MOTOR A is being used to rotate the sonar
		DifferentialPilot pilot = new DifferentialPilot(5.6f, 11.2f, Motor.B, Motor.A); // (wheel diameter, dist between wheels, left motor, right motor )
		sonar = new UltrasonicSensor(SensorPort.S4);

		LCD.drawString("READY", 0, 10);
		while (true) {
			try {
				byte cmd = dis.readByte();
				LCD.drawInt(cmd,1,0,0);
				int param = dis.readInt();

				switch (cmd) {
				case TRAVEL: 
					pilot.travel(param);
					dos.writeInt(0);
					break;
				case ROTATE: 
					pilot.rotate(param);
					dos.writeInt(0);
					break;
				case FULL_SCAN:
					String values = fullscan();
					dos.writeUTF(values);
					break;	
				case SINGLE_SCAN:
					int val = sonar.getDistance();
					System.out.println(""+ val);
					dos.writeInt(val);
					break;
				case EXIT:
					System.out.println("close");
					dis.close();
					dos.close();
					btc.close();
					System.exit(0);
				default:
					dos.writeInt(-1);
				}
				dos.flush();
				
			} catch (Exception e) {
				System.err.println("FERROU "+ e);
				Thread.sleep(2000);
				System.exit(1);
			}
		}
	}
	public static String fullscan() throws IOException{ //teste do método
		int val = 0;
		//Motor.C.setSpeed(100);
		String values = "I";
		values += "|";
		for (int i=0; i <= 300; i++){
			val = sonar.getDistance();
		    values += val;
		    values += "|";
		    Delay.msDelay(100);
		    System.out.println(val);
		}
		values += "E";
		return values;
	}
}
