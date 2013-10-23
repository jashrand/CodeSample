///////////////////////////////////////////////////////////////////////////////
//                               Constants.java                              //
///////////////////////////////////////////////////////////////////////////////

//This class defines constant static variables that can be used
//anywhere within the program
public class Constants
{

  //Full safe radius of where we can put robot, in meters.
  public final static float OBSTACLE_SAFETY_RADIUS = 0.26f;
 
 // area considered the map
 public final static float MAP_XMIN = 0.0f;
 public final static float MAP_XMAX = 100.0f;
 public final static float MAP_YMIN = 0.0f;
 public final static float MAP_YMAX = 100.0f;
 
 // area of the endzone
 public final static float ENDZONE_XMIN = 0.0f;
 public final static float ENDZONE_XMAX = 0.12f;
 
 public final static float ENDZONE_YMIN = 0.5f;//1.11f;
 public final static float ENDZONE_YMAX = 2.50f; 
 
// robot start pose
public final static float ROBOT_X = 460.0f;
public final static float ROBOT_Y = 330.0f;

// friction constants
public final static float RUG_TURN_FRICTION = 1.00f;
public final static float RUG_DRIVE_FRICTION = 1.1f;

// wheel speeds during calibration
public final static float CALIBRATION_FORWARD_SPEED = 0.1f;
public final static float CALIBRATION_BACKWARD_SPEED = -0.5f;
// new coordinate after calibration
public final static float CALIBRATION_RESET_COORDINATE = 6.0f;

// distance from IR sensors to center
public final static float IR_TO_CENTER = 113.0f;

// whether to calibrate x or y
public final static boolean CALIBRATE_X_POSE = true;
public final static boolean CALIBRATE_Y_POSE = false;

// what value is large enough to show ohmm is turning
public final static float SIGNIFICANT_TURNING = 20.0f;

// distance to go to hit a wall
public final static float X_CALIBRATION_DIST = 400.0f;

// distance from the endzone wall to place the targets
 public final static float ENDZONE_WALL_BUFFER = 300.0f;
 
  public final static boolean DEBUG = true;
  
  //Constructor (Not really needed)
  public Constants(){}
}
