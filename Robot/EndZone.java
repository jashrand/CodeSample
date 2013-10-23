///////////////////////////////////////////////////////////////////////////////
//                               EndZone.java                                //
///////////////////////////////////////////////////////////////////////////////

import java.util.*;

//Represents an endzone, the area where the robot will place the targets
public class EndZone extends Parallelogram{

   //Constructor
   public EndZone(float xmin, float xmax, float ymin, float ymax)
   {
		super(xmin, xmax, ymin, ymax);
   }
    


   public ArrayList<Point> generateGoals(int goalNum){
        System.out.println("goalNum: " + goalNum);	
	if(goalNum >= 1) {
			ArrayList<Point> goalPoints = new ArrayList<Point>();
			float x = (Constants.ENDZONE_XMIN + Constants.ENDZONE_XMAX) / 2;
			
			//divide endzone into equal parts, to place each goalPoint inbetween each part
			float firstPartWidth = (Constants.ENDZONE_YMAX - Constants.ENDZONE_YMIN) / (goalNum + 1);
			
			// loop through all the parts so all goalPoints are equally spaced
			float y;
			for (int i=1; i<=goalNum; i++) {
				// calculate y coordinate
				y = (firstPartWidth * i) + Constants.ENDZONE_YMIN;
				goalPoints.add(new Point(x, y));
			}
		System.out.println("Goal points: " + goalPoints);	
			return goalPoints;
		}
		else {
			throw new RuntimeException("No goals to generate");
		}
	}

}
