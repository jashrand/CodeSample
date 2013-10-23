///////////////////////////////////////////////////////////////////////////////
//                               Parallelogram.java                          //
///////////////////////////////////////////////////////////////////////////////
//Represents a 2D parallelogram in WORLD frame
import java.util.*;

public class Parallelogram{
  
    protected float xmin, xmax, ymin, ymax;
    protected List<Point> bounds = new ArrayList<Point>();
    
    //Constructor
    public Parallelogram(float xmin, float xmax, float ymin, float ymax){
        this.xmin = xmin; 
        this.xmax = xmax; 
        this.ymin = ymin; 
        this.ymax = ymax;
        
        float R = Constants.OBSTACLE_SAFETY_RADIUS;
        
        bounds.add(new Point(xmin - R, ymin - R));
        bounds.add(new Point(xmin - R, ymax + R));
        bounds.add(new Point(xmax + R, ymax + R));
        bounds.add(new Point(xmax + R, ymin - R));
    }
    
    /**
     * Return a list of nodes representing the expanded bounds of the obstacle
     **/

    public List<Node> getNodes(){
        List<Node> nodes = new ArrayList<Node>();
        for(Point p: bounds){
            nodes.add(new Node(p));
        }
        return nodes;
    }
   
   public float getWidth()
   {
      return this.xmax - this.xmin;
   }
   
   public float getHeight()
   {
      return this.ymax - this.ymin;
   }
   
    public String toString(){
        return String.format("X-min: %f X-max: %f Y-min: %f Y-max: %f", xmin, xmax, ymin, ymax);   
    }
	
	
	

}
