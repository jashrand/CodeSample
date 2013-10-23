///////////////////////////////////////////////////////////////////////////////
//                               Obstacle.java                               //
///////////////////////////////////////////////////////////////////////////////

//This class represents an obstacle in world frame, as provided by the map file

public class Obstacle extends Parallelogram{
    
    //Constructor
    public Obstacle(float xmin, float xmax, float ymin, float ymax){
        super(xmin, xmax, ymin, ymax);

        if(Constants.DEBUG)
        {
          System.out.println("OBSTACLE BOUNDS: ");
          for(Point b : bounds){
             System.out.println(b.toString());
          }
        }
    }

    
    /**
     * Determines whether a line segment intersects the obstacle
     **/
    public boolean intersects(Point p1, Point p2){
        return sat(p1,p2, bounds.get(0), bounds.get(1), bounds.get(2), bounds.get(3));
    }
    
    /**
     * AArect - lineSeg intersection
     **/
    public boolean sat(Point lp1, Point lp2, Point rp1, Point rp2, Point rp3, Point rp4){
        //segment center
        float lcx = (lp1.getX() + lp2.getX()) / 2.0f;
        float lcy = (lp1.getY() + lp2.getY()) / 2.0f;
        
        //segment tag vector from center to endpoint
        float ltx = lp1.getX() - lcx;
        float lty = lp1.getY() - lcy;
        
        //rect dims
        float rw = Math.abs(rp3.getX() - rp2.getX());
        float rh = Math.abs(rp2.getY() - rp1.getY());
        
        //rect center
        float rcx = (rp3.getX() + rp2.getX()) / 2.0f;
        float rcy = (rp2.getY() + rp1.getY()) / 2.0f;
        
        //segment tangent dir
        float rdx = rw / 2.0f;
        float rdy = rh / 2.0f;
        
        // xform rect center to origin
        lcx = lcx-rcx;
        lcy = lcy-rcy;
        rcx = 0f;
        rcy = 0f;
        
        // segment normal
        float lnx = -lty;
        float lny = ltx;
        
        // test three separating axes
        if (rdx - 0.0001f <= (Math.abs(lcx) - Math.abs(ltx))){
            return false; // x axis (not intersecting)
        }
        else if (rdy - 0.0001f <= (Math.abs(lcy) - Math.abs(lty))){
            return false; // y axis (not intersecting)
        }
        else if (( (rdx*Math.abs(lnx)) + (rdy*Math.abs(lny)) ) - 0.0001f <= 
                 (Math.abs( (lcx*lnx)+(lcy*lny) ) - Math.abs( (ltx*lnx)+(lty*lny) ))){
            return false; // segment normal axis (not intersecting)
        }
        return true;
    }
}
