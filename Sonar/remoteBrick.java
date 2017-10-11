import lejos.robotics.navigation.DifferentialPilot;
import lejos.nxt.Motor;
import lejos.util.Delay;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;

public class remoteBrick {
    static DifferentialPilot pilot;
    static UltrasonicSensor sonar;

    private static final byte TRAVEL = 0; 
    private static final byte ROTATE = 1;
    private static final byte FULL_SCAN = 2; 
    private static final byte SINGLE_SCAN = 3; 
    private static final byte EXIT = 4;

    public static void main(String[] args){
        Delay.msDelay(2000);
        byte cmd = 0; int param = 0;
        pilot = new DifferentialPilot(5.6f, 11.2f, Motor.B, Motor.A);
        sonar = new UltrasonicSensor(SensorPort.S4);
        while(true) {
            System.out.print("Enter command [0:TRAVEL 1:ROTATE 2:FULL_SCAN 3:SINGLE_SCAN 4:EXIT]: ");
            cmd = (byte) scan.nextFloat(); 
            if (cmd == 0 || cmd == 1){
                System.out.println("Enter integer parameter: ");
                param = scan.nextInt();
            }
            switch (cmd) {
                case TRAVEL: 
                    pilot.travel(param);
                    break;
                case ROTATE: 
                    pilot.rotate(param);
                    break;
                case FULL_SCAN:
                    fullscan();
                    break;  
                case SINGLE_SCAN:
                    int val = sonar.getDistance();
                    System.out.println(""+ val);
                    break;
                case EXIT:
                    System.out.println("close");
                    System.exit(0);
                default:
                    System.out.println("default");
            }
        }
    }

    public static void fullscan(){ //teste do método
        int val = 0;
        for (int i=0; i <= 900; i+=10){
            Motor.C.rotate(10);
            val = sonar.getDistance();
            System.out.println(val);
        }
    }

}
