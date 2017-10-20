import lejos.pc.comm.*;
import java.util.Scanner;
import java.io.*;
import lejos.util.Delay;
import java.util.ArrayList;

public class MasterSonar {
	private static final byte TRAVEL = 0; 
	private static final byte ROTATE = 1;
	private static final byte FULL_SCAN = 2; 
	private static final byte SINGLE_SCAN = 3; 
	private static final byte EXIT = 4; 
	
	private NXTComm nxtComm;
	//private DataOutputStream dos;
	//private DataInputStream dis;	
	private DataOutputStream dos;
	private DataInputStream dis;

	
	private static final String NXT_ID = "PKSM"; // NXT BRICK ID

	private String sendCommand(byte command) throws IOException{ //working
		try {
			dos.writeByte(command);
			dos.writeInt(0);
			dos.flush();

		} catch (IOException ioe) {
			System.err.println("IO Exception "+ ioe);
			System.exit(1);
			return "Error";
		}
		return dis.readUTF();
	}



	private ArrayList<Integer> sendCommand(byte command, int param) { // used by functions 0 and 1
		ArrayList<Integer> list = new ArrayList<>();
		try {
			dos.writeByte(command);
			dos.writeInt(param);
			dos.flush();
			list.add(dis.readInt());
		} catch (IOException ioe) {
			System.err.println("IO Exception "+ ioe);
			System.exit(1);
		}
		return list;
	}

	private void connect() throws IOException {
		try {
			NXTComm nxtComm = NXTCommFactory.createNXTComm(NXTCommFactory.USB);
			/* Uncomment next line for Bluetooth communication */
			//NXTComm nxtComm = NXTCommFactory.createNXTComm(NXTCommFactory.BLUETOOTH);			
			NXTInfo[] nxtInfo = nxtComm.search(MasterSonar.NXT_ID);
			
			if (nxtInfo.length == 0) {
				System.err.println("NO NXT found");
				System.exit(1);
			}
			
			if (!nxtComm.open(nxtInfo[0])) {
				System.err.println("Failed to open NXT");
				System.exit(1);
			}
			
			dis = new DataInputStream(nxtComm.getInputStream());
			dos = new DataOutputStream(nxtComm.getOutputStream());
			
		} catch (NXTCommException e) {
			System.err.println("NXTComm Exception: "  + e.getMessage());
			System.exit(1);
		}
	}		

	// private void close() {
	// 	try {
	// 		//System.out.println("byee");
	// 		dos.writeByte(EXIT);
	// 		dos.writeInt(0);
	// 		dos.flush();
	// 		Thread.sleep(400);
	// 		//System.out.println("bcloss");
	// 		nxtComm.close();
	// 		dis.close();
	// 		dos.close();
	// 		//System.exit(0);
	// 	} catch (Exception ioe) {
	// 		System.err.println("IO Exception Paula " + ioe);
	// 	}
	// }	
	public static void main(String[] args) throws IOException{
		byte cmd = 0; int param = 0; String s=""; 
		ArrayList<Integer> ret ;
		//float ret=0f; float readings; float singleRead = -1;
		MasterSonar master = new MasterSonar();
		master.connect();
	    Scanner scan = new Scanner( System.in );	    
	    while(true) {
	    	System.out.print("Enter command [0:TRAVEL 1:ROTATE 2:FULL_SCAN 3:SINGLE_SCAN 4:EXIT]: ");
	    	cmd = (byte) scan.nextFloat(); 
	    	if (cmd == 0 || cmd == 1){
	    		System.out.println("Enter integer parameter: ");
	    		param = scan.nextInt();
	    		ret = master.sendCommand(cmd, param); 
	    		System.out.println("cmd: " + cmd + " Parameter: " + param +" return: " + ret.toString());
	    		param = 0;
	    	} 
	    	else if (cmd == 2){
	    		s = master.sendCommand(cmd);
	    		System.out.println("cmd: " + cmd + " Sonar Values " + s);
	    	}
	    	else{
	    		ret = master.sendCommand(cmd,param);
	    		System.out.println("Value: "+ ret.toString());
	    	}
	    }
	}

}
