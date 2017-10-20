 import lejos.nxt.Button;
 import lejos.nxt.LCD;
 import lejos.nxt.*;
 import lejos.nxt.comm.RConsole;
 /*
 * Programna 1
 */
public class MotorTutor1
 {
      public static void main(String[] args)
      {
           RConsole.openAny(0);
           UltrasonicSensor sonar = new UltrasonicSensor(SensorPort.S4);
           LCD.drawString("Programa 1", 0, 0);
           Button.waitForAnyPress();
           LCD.clear();
           Motor.C.setSpeed(100);
           RConsole.println("Inicial "+ Motor.C.getTachoCount() );
           Motor.C.rotate(450);
           RConsole.println("Metade da direita "+ Motor.C.getTachoCount() + " " + sonar.getDistance());
           Motor.C.rotate(-900);
           RConsole.println("Metade da esquerda "+ Motor.C.getTachoCount() + " " + sonar.getDistance() );
           Motor.C.rotate(450);
           RConsole.println("Meio "+ Motor.C.getTachoCount() + " " + sonar.getDistance());
           //Motor.C.resetTachoCount();
           RConsole.println("Meio depois do reset "+ Motor.C.getTachoCount()+ " " + sonar.getDistance() );
           Motor.C.rotate(450);
           RConsole.println("Metade da direita "+ Motor.C.getTachoCount() + " " + sonar.getDistance() );
           Motor.C.rotate(-900);
           RConsole.println("Metade da esquerda "+ Motor.C.getTachoCount()  + " " + sonar.getDistance());
           Motor.C.rotate(450);
           RConsole.println("Meio "+ Motor.C.getTachoCount() + " " + sonar.getDistance() );
           //Motor.C.rotate(900);
           RConsole.close();
     
      }
 }