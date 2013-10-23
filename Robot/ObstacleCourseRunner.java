////////////////////////////////////////////////////////////////////////////////
//                         ObstacleCourseRunner.java                          //
////////////////////////////////////////////////////////////////////////////////
//  This file contains the main method that will run the obstacle course.     //
//  ObstacleCourseRunner keeps track of the state of the robot at the highest //
//  level, and makes calls to other objects to handle the details.            //
////////////////////////////////////////////////////////////////////////////////
import ohmm.*;
import static ohmm.OHMM.*;
import ohmm.OHMM.AnalogChannel;
import static ohmm.OHMM.AnalogChannel.*;
import ohmm.OHMM.DigitalPin;
import static ohmm.OHMM.DigitalPin.*;

import java.util.ArrayList;
import java.lang.Exception;

public class ObstacleCourseRunner implements Runnable{

	private OHMMDrive ohmm;
	private static final float ROBOT_SPEED = (float)50;

////////////////////////////////////////////////////////////////////////////////
                    
                     ////////////////////
                     //   STATE ENUM   //
                     ////////////////////
    
    public static enum STATE{
      INITIALIZE_CALIBRATE, //Initial calibration phase
      DRIVE_TO_TARGET,     //Driving to a lollipop target
      GRAB_TARGET,         //Grab the target with gripper
      RETURN_WITH_TARGET,   //Return to goal area with target
      CALIBRATE_Y,	   // calibrates in the Y axis
      PLACE_TARGET,        //Place the target in the goal area
      CALIBRATE_X,    //calibratse in the X axis
      FINISH               //Finish up!  Return to the home pose, dance, whatever.
    };
	
	private STATE state; // the current state
	 
//	private static 
        //The obstacles provided by the map file
        private ArrayList<Obstacle> obstacles;

        //The targets provided by map file   
        private ArrayList<Point> targets;
		
	//The final location for all the targets
	private ArrayList<Point> goals;
    
////////////////////////////////////////////////////////////////////////////////

    private Robot robot;
    
    
    //Classes that handle different parts of the challenge
    private GlobalNavigation globalNav; //Handles global navigation and path following
    private ObjectDetect objectDetect;  //Handles Image processing / Object detection
  
    //Debug map server
    private ServerRunner mapServerRunner;

    //Thread to handle the image processing
    private Thread cameraThread;

    private Thread mapThread; //Thread to handle debug map server 

    private ArmControl armControl;
////////////////////////////////////////////////////////////////////////////////
    
////////////////////    
//  Constructor   //
////////////////////
    public ObstacleCourseRunner(Robot robot, String mapFileName, ObjectDetect objectDetect)
    {
        this.robot = robot;

	//PARSE THE MAP FILE
        MapParser parser = new MapParser(mapFileName);
        parser.parse();

        this.obstacles = parser.getObstacles();
        this.targets = parser.getTargets();

		//Sort targets by distance
		ArrayList<Point> temp_arr = new ArrayList<Point>();
		Point temp_min = new Point(0, 0);
		float min_dist = (float)100000.0;
		float last_min_dist = 0;
		for (Point i: this.targets)
		{
			min_dist = (float)100000.0;
			for (Point p: this.targets)
			{
				if ((Math.sqrt((Math.pow(p.getX() - Constants.ROBOT_X, 2)) + (Math.pow(p.getY() - Constants.ROBOT_Y, 2))) <= min_dist) && (Math.sqrt((Math.pow(p.getX() - Constants.ROBOT_X, 2)) + (Math.pow(p.getY() - Constants.ROBOT_Y, 2))) >= last_min_dist))
				{
					temp_min = p;
					min_dist = (float)Math.sqrt(Math.pow((p.getX() - Constants.ROBOT_X), 2) + Math.pow((p.getY() - Constants.ROBOT_Y), 2));
				}
			}
			last_min_dist = min_dist;
			temp_arr.add(temp_min);
		}
		this.targets = temp_arr; //TODO: Make sure this works

	 // calculate the goals ArrayList
        EndZone endzone = new EndZone(Constants.ENDZONE_XMIN, Constants.ENDZONE_XMAX, Constants.ENDZONE_YMIN, Constants.ENDZONE_YMAX);
         //System.out.println("Targets: " + this.targets.size());
         this.goals = endzone.generateGoals(this.targets.size());
	System.out.println("===================================");
	System.out.println("Goals: " + goals);
	System.out.println("=====================================");
	System.out.println("First Goal: " + goals.get(0));
	System.out.println("=====================================");


        //Object Detection camera stuff should be handled in separate thread
       this.objectDetect = objectDetect;
                                           
        init();
    }
    
////////////////////
//   INITIALIZE   //
////////////////////
    public void init()
    {
        
	this.state = STATE.INITIALIZE_CALIBRATE; //Set the state
      
        try{
           this.armControl = new ArmControl(this.robot); 
           this.armControl.init();
        }
        catch(Exception e){
           System.out.println("Error initializing ArmControl");
           e.printStackTrace();
        }
        this.globalNav = new GlobalNavigation(this.robot); //Initialize Global Navigation

       globalNav.setGoal(this.targets.get(0)); //Set target to first in arraylist
       for(Obstacle o : this.obstacles)  //Add obstacles
       {
         globalNav.addObstacle(o);
        }

       //Set OHMM start coordinates
       this.robot.driveSetPose(Constants.ROBOT_X, Constants.ROBOT_Y, 0.0f);

}
    
    
//////////////////////////
//      BUMP CODE       //
//////////////////////////
    private Boolean getLeftBump()
	{
        return ohmm.senseReadDigital(IO_A0); //left bump
    }
    private Boolean getRightBump()
	{ 
        return ohmm.senseReadDigital(IO_A1); //right bump 
    }

////////////////////////
//    NEXT TARGET     //
////////////////////////
private void nextTarget(){
   if(this.targets.size() > 0)
   {
     this.state = STATE.DRIVE_TO_TARGET;
   }
   else
   {
      System.out.println("Found all the targets!");
      this.state = STATE.FINISH;
   }
}


////////////////////
//      RUN       //
////////////////////
    public void run()
    {
		while(true) {
			 if(this.state == STATE.INITIALIZE_CALIBRATE) 	
			{
                    //Initial calibration.  Wait for user to click in web viewer, and detect blob.
                    
                             while(!this.objectDetect.isCalibrated())
                             {
                                try{
                                   Thread.sleep(100); //Sleep 1/10 of a second
                                } 
                                catch(InterruptedException e){
                                  System.out.println("Caught interrupted exception");
                                }
                             }

                            this.state = STATE.DRIVE_TO_TARGET;
			}
			else if(this.state == STATE.DRIVE_TO_TARGET)
			{
        	            System.out.println("DRIVING TO TARGET");
                           //Stow arm
                           armControl.carryPose();
                	    Point target = this.targets.get(0);
			   this.globalNav.setStart(new Point(robot.driveGetPose()[0] / 1000.0f, robot.driveGetPose()[1] / 1000.0f)); 
                           this.globalNav.setGoal(target); //Set the target
        	            this.globalNav.calculatePath();
                	    GlobalNavigation.driveNearTarget(this.robot, this.globalNav);
                  		 //At this point, the bot should be very close to the target.
                  		this.state = STATE.GRAB_TARGET;
			}
			else if(this.state == STATE.GRAB_TARGET)
			{
      			 	System.out.println("GRABBING TARGET");

                                //This thread will run the VisualServo code until
                                //the target is grabbed succesfully.
                                objectDetect.enterServoLoop();

                                System.out.println("Returned from Servo loop");
  
                                this.state = STATE.RETURN_WITH_TARGET;
			}
			else if(this.state == STATE.RETURN_WITH_TARGET)
			{
                               System.out.println("DRIVING TO GOAL");
                               float[] curPose = robot.driveGetPose();
                               System.out.println("My pose is : ["  + curPose[0] + ", " + curPose[1] + ", " + curPose[2] +"]!");
                               //Stow arm
                               this.armControl.carryPose();
					// go to specific place on the map
					// make method to go towards closest target after they have all been listed
					//g.getFirstTarget();
					Point goal = this.goals.get(0);


                                        System.out.println("GOAL SET TO: " + goal.toString());
					this.globalNav.setStart(new Point(curPose[0]/1000.0f, curPose[1]/1000.0f));
                                        this.globalNav.setGoal(goal); //Set the target
					this.globalNav.calculatePath();
					GlobalNavigation.followPath(this.robot, this.globalNav);
  	             	// turn and face the wall
                //     		this.globalNav.faceDirection(this.robot,(float)Math.PI);	
                        //At this point, the bot should at the goal.
                                        robot.turnToTheta((float)Math.PI);
                                        GlobalNavigation.blockForQueue(robot);
                                        this.state = STATE.CALIBRATE_X;
			}
                        else if(this.state == STATE.CALIBRATE_X)
      {
          System.out.println("Calibrating X");
          GlobalNavigation.calibrate(this.robot, Constants.CALIBRATE_X_POSE);
          // put the target done and delete it from global list
          // GlobalNavigation.placeTarget();
          this.state = STATE.PLACE_TARGET;
      }
			else if(this.state == STATE.PLACE_TARGET)
			{
					System.out.println("PLACING target");
                   this.targets.remove(0);
                  this.goals.remove(0);
				this.state = STATE.CALIBRATE_Y;
			}
			else if(this.state == STATE.CALIBRATE_Y)
			{
				System.out.println("Calibrating Y");
                            //drive to wall
				robot.turnToTheta((float)(-0.5 * Math.PI));
				GlobalNavigation.blockForQueue(robot);
				GlobalNavigation.maintainXPose(robot);
				GlobalNavigation.calibrate(this.robot, Constants.CALIBRATE_Y_POSE);
                            
                           nextTarget();
			}
			
		else if(this.state == STATE.FINISH)
			{
                               break;
				// makes it dance if there is time
        //move to center, spin in circles,raise and lower arm
				
				/*while(true) {
					ArmControl.goCrazy();
					robot.turnToTheta((float)((Math.PI / 2) + robot.driveGetPose()[2]);	
					System.out.println("DANCE DANCE <(-_-)> !!!!!");
				}*/
			}   
		}
	   System.out.println("Done.");
       System.exit(0);
    }

////////////////////////////////////////////////////////////////////////////////
//                                 MAIN                                       //
////////////////////////////////////////////////////////////////////////////////
    public static void main(String[] args)
    {
        //Make sure user provided map file		
        if(args.length == 0){
            System.out.println("Need to specify map file!");
            System.exit(0);
        }


        //Main state thread
	ObstacleCourseRunner runner;
	try{
           //Robot robot = (Robot)OHMM.makeOHMM(new String[]{"-r", "/dev/ttyACM1"});
           Robot robot = new Robot("/dev/ttyACM1"); 
  
         
        VisualServo servo = new VisualServo(robot);
           ObjectDetect objectDetect = new ObjectDetect(robot, servo);
           runner = new ObstacleCourseRunner(robot, args[0], objectDetect);

           Thread obstacleThread = new Thread(runner);
           obstacleThread.start();

           objectDetect.init();
           objectDetect.mainLoop();
           objectDetect.release();
        }
        catch(Exception e){
           System.out.println("Error initializing OHMM.");
           e.printStackTrace();
           System.exit(0);
        }
        
        
    }
}
