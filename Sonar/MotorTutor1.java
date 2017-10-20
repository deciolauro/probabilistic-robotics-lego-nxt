 import lejos.nxt.Button;
 import lejos.nxt.LCD;
 import lejos.nxt.Motor;
 /*
 * Programna 1
 */
public class MotorTutor1
 {
      public static void main(String[] args)
      {
           LCD.drawString("Programa 1", 0, 0);
           Button.waitForAnyPress();
           LCD.clear();
           Motor.C.setSpeed(40);
           System.out.println("Inicial "+ Motor.C.getTachoCount() );
           Motor.C.rotate(450);
           System.out.println("Metade da direita "+ Motor.C.getTachoCount() );
           Motor.C.rotate(-900);
           System.out.println("Metade da esquerda "+ Motor.C.getTachoCount() );
           Motor.C.rotate(450);
           System.out.println("Meio "+ Motor.C.getTachoCount() );
           //Motor.C.resetTachoCount();
           System.out.println("Meio depois do reset "+ Motor.C.getTachoCount() );
           Motor.C.rotate(450);
           System.out.println("Metade da direita "+ Motor.C.getTachoCount() );
           Motor.C.rotate(-900);
           System.out.println("Metade da esquerda "+ Motor.C.getTachoCount() );
           Motor.C.rotate(450);
           System.out.println("Meio "+ Motor.C.getTachoCount() );
           //Motor.C.rotate(900);
     
      }
 }