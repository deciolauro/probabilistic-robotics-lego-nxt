import java.io.ObjectOutputStream;
import java.io.ObjectInputStream;
import java.io.*;

public class array {

   public static void main(String args[])throws Exception {

		FileOutputStream fos = new FileOutputStream("data.txt");
		ObjectOutputStream oos = new ObjectOutputStream(fos);

		double[] values = new double[6];
		int index = 0;
		for (int i=30; i <= 180; i+=30){
			values[index] = i;
			index++;
		}
		oos.writeObject(values);
        oos.flush();
        oos.close();

        FileInputStream fis = new FileInputStream("data.txt");
        ObjectInputStream ois = new ObjectInputStream(fis);

        //double[] readingVal = new double[6];
        double[] readingVal = (double[]) ois.readObject();
        ois.close();
        int j=0;
        while (j < readingVal.length){
        	System.out.println("" + readingVal[j]);
        	j++;
        }
   }
}