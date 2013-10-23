///////////////////////////////////////////////////////////////////////////////
//                               Point.java                                  //
///////////////////////////////////////////////////////////////////////////////
//Represents a 2D point in WORLD frame
public class Point{
   //World x-y coordinates
   private final float x;
   private final float y;


   //Constructor
   public Point(float x, float y)
   {
     this.x = x;
     this.y = y;
   }

   public float getX()
   {
      return this.x;
   }

   public float getY()
   {
      return this.y;
   }

   //Calculate distance between this point and other point p
   public float distance(Point p)
   {
       return (float)(Math.sqrt(((x - p.getX())*(x - p.getX())) + 
                                ((y - p.getY())*(y - p.getY()))));

   }
    
    public String toString(){
        return String.format("x: %f y: %f ", x, y);   
    }

   public double angleTo(Point other)
   {
     //TODO
       return 0.0f;
   }

}
