import lejos.nxt.Button;
import lejos.nxt.LightSensor;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.nxt.SensorPort;

public class unregulatedMotor {
	
	static LightSensor light;
	static NXTMotor mB;
	static NXTMotor mC;
	
	public static void main(String args[])  
    	{
		light = new LightSensor(SensorPort.S4);
		mB = new NXTMotor(MotorPort.B);
		mC = new NXTMotor(MotorPort.C);
		Button.waitForAnyPress();
		while (light.getLightValue() < 50){
			mB.setPower(50);
			mC.setPower(50);
			//mB.setPower(-50);
			//mC.setPower(-50);
		}
		mB.stop();
		mC.stop();
    	}	
}
