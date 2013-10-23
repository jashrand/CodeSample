import java.util.ArrayList;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.lang.Exception;


//This class is used for parsing a mapfile, provided the map file name.
public class MapParser{


   private String fileName;
   private ArrayList<Obstacle> obstacles;
   private ArrayList<Point> targets;
   private boolean arena_found;
   //Constructor
   public MapParser(String fileName)
   {
      this.fileName = fileName;     
      this.obstacles = new ArrayList<Obstacle>();
      this.targets = new ArrayList<Point>();
	  boolean arena_found = false;
      System.out.println("Using map file: " + fileName);
   }


   //Parse the file!
   public void parse()
   {
     ArrayList<String> lines = new ArrayList<String>();
     
     try{
        //Populate lines ArrayList with the file contents
        BufferedReader in = new BufferedReader(new FileReader(fileName));
        while(true){  //Loop until end of file

           String line = in.readLine();
           if(line != null)
           {
              lines.add(line);
           }
           else
           {
              break;
           }
        }

        in.close();
     }
     catch(Exception e){
        System.out.println("Error reading file!");
        e.printStackTrace();
    }

    System.out.println("MAP FILE: ");
    for(String line : lines)
    {
       String[] sLine = line.split(" ");
       if(sLine.length == 2)  //Target
       {
          float x = Float.parseFloat(sLine[0]);
          float y = Float.parseFloat(sLine[1]);
          Point targ = new Point(x, y);
          targets.add(targ);
       }
       else if((sLine.length == 4) && (arena_found)) //Obstacle
       {
          float xmin = Float.parseFloat(sLine[0]);
          float xmax = Float.parseFloat(sLine[1]);
          float ymin = Float.parseFloat(sLine[2]);
          float ymax = Float.parseFloat(sLine[3]);
          
          Obstacle obs = new Obstacle(xmin, xmax, ymin, ymax);
          obstacles.add(obs);

       }
       else if(sLine.length == 4) //Arena
       {
          float xmin = Float.parseFloat(sLine[0]);
          float xmax = Float.parseFloat(sLine[1]);
          float ymin = Float.parseFloat(sLine[2]);
          float ymax = Float.parseFloat(sLine[3]);
          
		  // bottom wall
          Obstacle obs = new Obstacle((xmin - 1.0f), xmin, ymin, ymax);
          obstacles.add(obs);
		  // top wall
          obs = new Obstacle(xmax, (xmax + 1.0f), ymin, ymax);
          obstacles.add(obs);
		  // right wall
          obs = new Obstacle(xmin, xmax, (ymin - 1.0f), ymin);
          obstacles.add(obs);
		  // left wall
          obs = new Obstacle(xmin, xmax, ymax, (ymax + 1.0f));
          obstacles.add(obs);

		  arena_found = true;

       }
    }
   }

   //ACCESSORS
   public ArrayList<Point> getTargets()
   {
      return this.targets;
   }

  public ArrayList<Obstacle> getObstacles()
  {
     return this.obstacles;
  }

}
