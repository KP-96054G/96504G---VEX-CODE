import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class Read
{
    public static void main(String[] args)
    {
        try{
            File information = new File("info.txt");
            Scanner reader = new Scanner(information);
            while(reader.hasNextLine()){
                System.out.println(reader.nextLine());
            }
        }catch(FileNotFoundException e){
            System.out.println("An error occurred.");
            e.printStackTrace();
        }
    }
}