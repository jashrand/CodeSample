////////////////////////////////////////////////////////////////////////////////
//                         GlobalNavigation.java                              //
////////////////////////////////////////////////////////////////////////////////
//  This file contains the methods that implement target tracking by a Global //
//  Navigation. It keeps track of all targets and goes to them in order by    // 
//  closest to farthest.                                                      //
////////////////////////////////////////////////////////////////////////////////

import java.io.*;
import java.util.*;
import java.lang.Math;

import ohmm.*;
import static ohmm.OHMM.*;
import ohmm.OHMM.AnalogChannel;
import static ohmm.OHMM.AnalogChannel.*;
import ohmm.OHMM.DigitalPin;
import static ohmm.OHMM.DigitalPin.*;

////////////////////////////////////////////////////////////////////////////////


class GlobalNavigation{
    
	public static final float R = Constants.OBSTACLE_SAFETY_RADIUS; //full safe radius of where we can put robot

    private OHMM ohmm;
	
    private Point goal;
    private Point start;
    private Point[] goals = new Point[20]; //Can keep track of up to 20 goals
    
    public Node goalNode;
    public Node startNode;
	
    public List<Obstacle> obstacles = new ArrayList<Obstacle>();
    public List<Node> nodes = new ArrayList<Node>();
	public ArrayList<Point> endPositions = new ArrayList<Point>(); // the list of places to place the targets
    
    
    ////////////////////////////////////////////////////////////////////////////////
    
    ////////////////////    
    //  Constructor   //
    ////////////////////
    public GlobalNavigation(OHMM ohmm)
    {
	this.ohmm = ohmm;
    }
    
	
    ////////////////////////////////
    //       Public Methods       //
    ////////////////////////////////
    
    //SETTERS
    public void setGoal(Point g)
    {
        this.goal = g;   
    }

    public void setStart(Point g)
    {
        this.start = g;
        this.startNode  = new Node(g);
        System.out.println("START SET TO: " + this.start.toString());
    }
    
    public void addObstacle(Obstacle obs)
    {
        this.obstacles.add(obs);
        
    }
    
    
    
    
    public void calculatePath(){
        findNodes();
        findEdges();
        findPath();
    }
    
    /**
     * Find shortest path to the goal Using Dijkstra's Algorithm
     **/
    public void findPath(){
        PriorityQueue<Node> q = new PriorityQueue<Node>();
        goalNode.visited = false;
        goalNode.distance = 0;
        q.add(goalNode);
        while(!q.isEmpty()){
            Node v = q.poll();
            if(!v.visited){
                for(Node n: v.neighbors){
                    float dist = v.distance + v.distance(n);
                    if(!n.visited && (n.distance > dist)){
                        n.next = v;
                        n.distance = dist;
                        q.add(n);
                    }
                }
            }
        }
    }

    
    /**
     * Constructs a visibility graph using known nodes and obstacles
     **/
    public void findEdges(){
        for(Node a: nodes){
            int index = nodes.indexOf(a) + 1;
            for(Node b: nodes.subList(index,nodes.size())){
                boolean hit = false;
                for(Obstacle o: obstacles){
                    if(o.intersects(a.point,b.point)){
                //        System.out.println("Hit!");
                //        System.out.println(a.toString());
                //        System.out.println(b.toString());
                //        System.out.println();
                        hit = true;
                        break;
                    }
                }
                if(!hit){
           //         System.out.println("adding neighbor");
           //         System.out.println(a.toString());
           //         System.out.println(b.toString());
           //         System.out.println();
                    a.addNeighbor(b);
                    b.addNeighbor(a);
                }
            }
        }
    }
    
    /**
     * Creates nodes using the start and end goals and the corners
     * of the obstacles
     **/
    public void findNodes(){ 
       //    for (int i = 0; i < goals.length; i++)
        //    {
        //       goalNode = new Node(goal); 
        //       nodes.add(goalNode);
        //   }
        //nodes.add(startNode);
        startNode = new Node(start);
        nodes.add(startNode);

        for(Obstacle o: obstacles){
            nodes.addAll(o.getNodes());
        }
        
        goalNode = new Node(goal);
        nodes.add(goalNode); //Create node for the goal
    }
    
    
    ///////////  STATIC METHODS FOR PATH-FOLLOWING //////////////////
    
    
    /**
     * standard motion planning. enqueues next command onto the queue at each node
     * of the graph to take into account current odometry
     **/
    private static void followPath2(Robot robot, 
                                    GlobalNavigation gn, boolean approach)
    
    {
        Node n = gn.startNode;
        if(n == null){
           System.out.println("start node is null for some reason.");
        }
        else
        {
          System.out.println("FOLLOWING PATH!");
          float[] cur = robot.driveGetPose();
          System.out.println("Initial  pose: " + cur[0] + ", " + cur[1] + ", " + cur[2]);
        while(n != gn.goalNode){
            if(n.next == null){
               System.out.println("ERROR:  NO PATH FROM START TO GOAL!");
              // break;
               System.out.println("Start Node: " + gn.startNode.toString());
            }
            robot.pollRobot();
            
            System.out.println("Next point: " + n.next);
            Point current = new Point(robot.driveGetPose()[0] / 1000.0f,
                                      robot.driveGetPose()[1] / 1000.0f);
            
            float result = n.next.point.getY() - current.getY();
            float t = (float)Math.atan2(n.next.point.getY() - current.getY(),
                                        n.next.point.getX() - current.getX());
            
         //   System.out.println("current: " + current.toString());
            System.out.println("Turning to theta: " + t + " radians");
          //  System.out.println(); 
            robot.turnToTheta(t);
            blockForQueue(robot);
            robot.pollRobot();
            
            //DIVIDE BY 1000 TO GET INTO METERS
            current = new Point(robot.driveGetPose()[0] / 1000.0f, //x
                                robot.driveGetPose()[1] / 1000.0f); //y
            
            float d = current.distance(n.next.point) * 1000.0f; //Now convert to mm
              System.out.println("Drive distance: " + d);
        //    System.out.println("Robot pose: x:" + robot.driveGetPose()[0] + " y:" + robot.driveGetPose()[1] + "mm");
            
            //The next node is the goal!
            if(n.next == gn.goalNode)
            {
                if(! approach){
                    robot.driveStraight(d);
                }//Drive right to the point
                else
                {
                    robot.driveStraight((float)(Math.max(0.0, d - 300)));   //Stop 0.3 m away from target
                    
                }
                
            }
            else{  //Just drive normally.
                System.out.println("Driving!");
                robot.driveStraight(d); 
            }
            blockForQueue(robot);
            n = n.next;
        }
        }
    }
    
    //Follow a path completely (Use to get to a specific location, such as the end zone)
    public static void followPath(Robot robot, GlobalNavigation gn)
    {
        followPath2(robot, gn, false);   
    }
    
    public static void driveNearTarget(Robot robot, GlobalNavigation gn)
    {
        followPath2(robot, gn, true); 
    }
    //Follow a path, but hold short a little at the end (Use for driving to targets, so we don't accidentally knock them over)
    
    
    /**
     * Block for the queue to empty, updating the view as the commands execute
     **/
    public static void blockForQueue(Robot robot){
        do {
            robot.pollRobot();
            try {
                Thread.sleep(100);
            }
            catch(InterruptedException e){
                System.err.print(e);
            }
        } while(robot.driveGetQueue() != 0);
    }
    
    /**
     * Commands robot to follow the calculated path. 
     * Queues all commands before hand
     **/
    public static void followPathPrecomputed(Robot robot, 
                                             GlobalNavigation gn){
        Node n = gn.startNode;
        float t = 0f;
        while(n != gn.goalNode){
            //turn to segment
            float tNew = (float)Math.atan2( n.next.point.getY() - n.point.getY(),n.next.point.getX() -n.point.getX()); 
            float tDelta = tNew - t;
            
            if(tDelta > Math.PI){
                tDelta = tDelta - (float)(2*Math.PI);
            }
            else if(tDelta < -Math.PI){
                tDelta = tDelta + (float)(2*Math.PI);
            }
            robot.driveTurn(tDelta);
            //drive segment
            float d = n.distance(n.next);
            robot.driveStraight(d);
            t = tNew;
            n = n.next;
        }
        blockForQueue(robot);
    }

// make the robot face a certain orientation
public static void faceDirection(Robot robot, float orientation) {
	robot.driveTurn(orientation - robot.driveGetPose()[2]);

}

//Use this to back up a little bit.  Useful for placing targets, and if an obstacle is hit
public static void backUp(Robot robot) {
        robot.driveStraight(-100.0f); //Move back 100 mm
        blockForQueue(robot);
}

	
	public static void calibrate(Robot robot, Boolean resetX){
		while(true) {
			// if both bumpers are triggered
			if (robot.senseReadDigital(IO_A0) && robot.senseReadDigital(IO_A1)){
				// stop both 
				robot.motSetVelCmd(0.0f, 0.0f);
				break;
			}
			// if neither bumbers are triggered
			else if (!robot.senseReadDigital(IO_A0) && !robot.senseReadDigital(IO_A1)){
				// drive forward 
				robot.motSetVelCmd(0.1f, 0.1f);
			}
			// if right bumper is triggered
			else if (robot.senseReadDigital(IO_A1)){
				// turn left
				robot.motSetVelCmd(0.1f, -0.05f); 
			}
			// if left bumper is triggered
			else {
				// turn right
				robot.motSetVelCmd(-0.05f, 0.1f);
			}
		}
		

        // reset x
		if(resetX) {
			
			robot.driveSetPose(6.0f, robot.driveGetPose()[1], robot.driveGetPose()[2]);
		}
		// reset y
		else{
			robot.driveSetPose(robot.driveGetPose()[0], 6.0f, robot.driveGetPose()[2]);
		}
		robot.driveStraight(-Constants.ENDZONE_WALL_BUFFER);
		blockForQueue(robot);
		
	}   
    
	public static void maintainXPose(Robot robot) {

	       robot.senseConfigAnalogIR(CH_2, 1);
	  	robot.senseConfigAnalogIR(CH_3, 1);
		robot.driveSetVW(50.0f, 0.0f);

		while(!(robot.senseReadDigital(IO_A0) && robot.senseReadDigital(IO_A1))) {
		
                     
                  
                    //If we are getting too  close to the wall, adjust a tiny bit
                   if(robot.senseReadAnalog(CH_2) < 200.0f || robot.senseReadAnalog(CH_3) < 200.0f){
						//Turn away from the wall ever so slightly
						robot.driveSetVW(50.0f, 0.05f);
					}
					else if(robot.senseReadAnalog(CH_2) > 500.0f || robot.senseReadAnalog(CH_3) > 500.0f)
					{
						robot.driveSetVW(50.0f, -0.05f);
					}
					else{  //we are in a good range
						robot.driveSetVW(50.0f, 0.00f);
					}

     }

		robot.driveSetVW(0.0f, 0.0f);

                //We have hit the side wall.  back up in y.  Use IR sensors for x
                robot.driveSetPose(robot.driveGetPose()[0], 0.0f, (float)(-Math.PI / 2.0));
                robot.driveStraight(-Constants.ENDZONE_WALL_BUFFER);
                 blockForQueue(robot);
        
        
        
        ///////////////////IR SENSOR AVERAGING //////////////////
               
               //Take some samples of the IR sensors
               float[] rearSamples = new float[20];
               float[] frontSamples = new float[20];
               for(int count = 0; count < 20; count++)
               {
                   frontSamples[count] = robot.senseReadAnalog(CH_2);
                   rearSamples[count] = robot.senseReadAnalog(CH_3);
               }

               //Find the mean of the data samples
        float frontMean = 0.0f, rearMean = 0.0f;
               for(int i = 0; i < 20; i++){
                   frontMean += frontSamples[i];
                   rearMean += rearSamples[i];
               }
        frontMean = frontMean / 20.0f;
        rearMean = rearMean / 20.0f;
        
        //Take the difference of each point from the mean to calculate variance
        float frontVariance= 0.0f, rearVariance = 0.0f;
        for(int i = 0; i <20; i++){
            frontVariance += (float)Math.pow((frontSamples[i] - frontMean), 2);
            rearVariance += (float)Math.pow((rearSamples[i] - rearMean), 2);
        }
        frontVariance = frontVariance / 20.0f; //Average result
        rearVariance = rearVariance / 20.0f;

        //Take standard deviation as square root of variance

        float frontDeviation = (float)Math.sqrt(frontVariance);
        float rearDeviation = (float)Math.sqrt(rearVariance);
        
        //Refine the data by excluding points beyond the standard deviation from the average calculation
        float frontRefinedAverage = 0.0f;
        int frontDataCount = 0; 
        float rearRefinedAverage = 0.0f;
        int rearDataCount = 0;
        for(int i = 0; i <20; i++){
            float frontDif = (float)Math.abs(frontSamples[i] - frontMean);
            float rearDif = (float)Math.abs(rearSamples[i] - rearMean);
            
            if(frontDif <= frontDeviation)
            {
                frontRefinedAverage += frontSamples[i];
                frontDataCount += 1;
            }
            if(rearDif <= rearDeviation){
                rearRefinedAverage += rearSamples[i];
                rearDataCount += 1;
            }
            
        }
        //Average the refined data
        frontRefinedAverage = (float)(frontRefinedAverage / frontDataCount);
        rearRefinedAverage = (float)(rearRefinedAverage / rearDataCount);
        
        float refinedAvgX = (float)((frontRefinedAverage + rearRefinedAverage) / 2.0f);
        float[] curPose = robot.driveGetPose();
        robot.driveSetPose(refinedAvgX + Constants.IR_TO_CENTER, curPose[1], (float)(-Math.PI /2.0));
        System.out.println("REFINED X ESTIMATE: " + refinedAvgX);

        ///////////////////////////////////////////////////////////
	}

}
