import java.io.DataOutputStream;
import java.io.DataInputStream;
import java.io.*;
//import 
public class string {

   public static void main(String args[])throws Exception {

		FileOutputStream fos = new FileOutputStream("data.txt");
		DataOutputStream dos = new DataOutputStream(fos);

		//double[] values = new double[6];
		int index = 0;
        //List<Character> values = new ArrayList<Character>();
        //values.add(index,'I');
        String values = "I";
        values += "|";
		for (int i=30; i <= 900; i+=30){
            values += i;
            values += "|";
		}
        values += "E";
		dos.writeChars(values);
        dos.flush();
        dos.close();

        FileInputStream fis = new FileInputStream("data.txt");
        DataInputStream dis = new DataInputStream(fis);

        //double[] readingVal = new double[6];
        //double[] readingVal = (double[]) dis.readObject();
        //ArrayList<Integer[]> list = new ArrayList<>();
        while(dis.available()>0) {
         
            // read character
            char c = dis.readChar();
            if (c == '|' )
                System.out.println("Barra" + c);
            else
                System.out.println(c);
         }
         dis.close();
        // int j=0;
        // while (j < readingVal.length){
        // 	System.out.println("" + readingVal[j]);
        // 	j++;
        // }
   }
}