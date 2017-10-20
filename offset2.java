import lejos.nxt.Button;
import lejos.nxt.Motor;
import lejos.nxt.UltrasonicSensor;
import lejos.nxt.SensorPort;
public class offset2 {

	public static void main(String[] args) {
		UltrasonicSensor sonar = new UltrasonicSensor(SensorPort.S4);
		//Button.waitForAnyPress();
		for (int i=0;i<=180;i+=10){
			Motor.C.rotate(10); //motor do sonar na porta C
			System.out.println(sonar.getDistance());				}
		Motor.C.rotateTo(0);		
	}
}
