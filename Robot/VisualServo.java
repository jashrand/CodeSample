/*******************************************************************************
*                                                                              *
*                                                                              *    
*                               VisualServo.java                               *
*                                                                              *
*******************************************************************************/


import com.googlecode.javacpp.*;
import com.googlecode.javacv.*;
import com.googlecode.javacv.cpp.*;
import static com.googlecode.javacv.cpp.opencv_calib3d.*;
import static com.googlecode.javacv.cpp.opencv_contrib.*;
import static com.googlecode.javacv.cpp.opencv_core.*;
import static com.googlecode.javacv.cpp.opencv_features2d.*;
import static com.googlecode.javacv.cpp.opencv_flann.*;
import static com.googlecode.javacv.cpp.opencv_highgui.*;
import static com.googlecode.javacv.cpp.opencv_imgproc.*;
import static com.googlecode.javacv.cpp.opencv_legacy.*;
import static com.googlecode.javacv.cpp.opencv_ml.*;
import static com.googlecode.javacv.cpp.opencv_objdetect.*;
import static com.googlecode.javacv.cpp.opencv_video.*;

import ohmm.*;
import static ohmm.OHMM.*;
import ohmm.OHMM.AnalogChannel;
import static ohmm.OHMM.AnalogChannel.*;
import ohmm.OHMM.DigitalPin;
import static ohmm.OHMM.DigitalPin.*;

import java.lang.Exception;


public class VisualServo implements Runnable{

    private OHMMDrive ohmm;
    public enum STATE{
       INIT,
       WAIT,
       TURN,
       DRIVE_FORWARD,
       GRAB,
       SEARCHING,
       VERIFY,
       RESET_ARM,
       RETURN_HOME,
       FINISHED
    }

    private Grasp grasp;
    
    private STATE state;
    private double turnAmt = 0.0; //How far we will turn (radians)
    private double driveAmt = 0.0;  //How far we will drive forward (meters)
    //Constructor
    public VisualServo(OHMM ohmm)
    {
         this.ohmm = (OHMMDrive)ohmm;
         this.state = STATE.INIT;
    }



    public void init(){
       this.state = STATE.INIT;
    }

    public synchronized STATE getState(){
        return this.state;
    }

    //Turn the given amount, in radians.
    //Positive amt:  Turns left.
    //Negative amt:  Turns right.
    public synchronized boolean isFinished(){
       return this.state == STATE.FINISHED;
    }

    public synchronized void turn(double amt) 
    {
         this.turnAmt = amt;
         this.state = STATE.TURN;
    }

    public synchronized void stopTurn()
    {
        if(this.state == STATE.TURN)
        { 
           this.state = STATE.WAIT;
           this.ohmm.driveSetVW((float)0.0, (float)0.0);
        } 
   }

   //Drive forward by the given distance, (meters)
   //Supplying a negative number will drive backwards
   public synchronized void driveForward(double dist)
   {
       this.driveAmt = dist;
       this.state = STATE.DRIVE_FORWARD;
   }


    public synchronized void grabSequence()
    {

        //If we are not already grabbing
        if(this.state != STATE.GRAB)
        {
           this.state = STATE.GRAB;
        }
    }

    public synchronized void search()
    {
       System.out.println("Searching for target...");
       this.state = STATE.SEARCHING;
    }


    public synchronized void verify()
    {
       System.out.println("Servo moving to verification pose");
       this.state = STATE.VERIFY;
    }
    
    public synchronized void resetArm()
    {
       this.state = STATE.RESET_ARM;
    }


    public synchronized void finish()
    {
        this.state = STATE.FINISHED;
        
    }
    
    public synchronized void openGripper()
    {
    	this.ohmm.armSetGripper((float)1.0); grasp.armWait(); 
    }
    
    public synchronized void closeGripper()
    {
        this.ohmm.armSetGripper((float)(-0.2)); grasp.armWait();
    	//this.ohmm.armSetGripper((float)0.0); grasp.armWait(); 
    }

    public void run()
    {
       while(true)
       {
          try{
          
             if(this.state == STATE.INIT) //Initialize grasp related stuff
             {
                 this.grasp = new Grasp(this.ohmm);
                
                 //Initializing grasp resets the pose, so we store it
                 float[] curPose = ohmm.driveGetPose();
                 this.grasp.init();
                 this.ohmm.driveSetPose(curPose[0], curPose[1], curPose[2]);       
		//		 this.grasp.cal();
                // this.grasp.home(false);	
                	 
                 this.state = STATE.WAIT;
                 float[] newPose = ohmm.driveGetPose();
               
             }
             else if(this.state == STATE.WAIT)
             {
                //System.out.println("WAITING");
                //Don't do anything, but check back in half a second to see
                //if state changed
                Thread.sleep(300);
               
             }

             else if(this.state == STATE.TURN)
             {
                float theta = this.ohmm.driveGetPose()[2];
 //               grasp.orientAndWait((float)(theta + this.turnAmt));
               
                ohmm.driveSetVW((float)0.0, (float)turnAmt);
                 //ohmm.driveTurn((float)turnAmt);
                //this.state = STATE.WAIT;
              
             }
             else if(this.state == STATE.DRIVE_FORWARD)
             {
                System.out.println("DRIVING");
                this.ohmm.driveStraight((float)driveAmt);
                this.grasp.driveWait();
                this.state = STATE.WAIT;  //Only drive a little bit at a time. 
              
             }
             else if(this.state == STATE.GRAB)
             {
                float[] robotPose = ohmm.driveGetPose();
               // this.grasp.graspDemo(robotPose[0], robotPose[1]);
               this.ohmm.driveStraight(-50.0f); //Drive backward 5 cm so we don't come down on top of the target
 
               this.grasp.interpolateArm((float)0.0, (float)(-50.0));
               this.grasp.armWait();
               this.ohmm.armSetGripper((float)1.0);
               this. grasp.armWait(); //Open gripper
               this.grasp.interpolateArm((float)50.0, (float)(0.0)); grasp.armWait();
          //To be extra sure that we got it, drive forward a bit more than we backed up
               this.ohmm.driveStraight((float)60.0); //6 cm
               this.grasp.driveWait();
               this.ohmm.armSetGripper((float)0.0); grasp.armWait();
               this.grasp.home(true); grasp.armWait(); //Go to home position
               System.out.println("Setting state to verify");
               this.state = STATE.VERIFY; //Next we need to verify
               
               //this.state = STATE.WAIT; //Go back to waiting
             }
             else if(this.state == STATE.SEARCHING)
             {
                 //Rotate counter-clockwise.  When the target is seen, ObjectDetect will
                 //change the servo state.
                 this.ohmm.driveSetVW((float)0.0, (float)0.1);
             }
             else if(this.state == STATE.VERIFY)
             {
                this.ohmm.driveSetVW((float)0.0, (float)0.0);
                this.grasp.home(true);
                this.grasp.armWait();
                this.grasp.interpolateArm((float)0.0, (float)(100.0)); //Move 100 up in z
                this.grasp.armWait();

                //Wait 1 second to prevent ObjectDetect from missing verification.  (There is a bit of a race condition here)
                Thread.sleep(1000);
 
                System.out.println("VERIFY IS OVER");
                this.state = STATE.WAIT; //Go back to waiting
              
                //this.state = STATE.RESET_ARM; //Go back
             }
             else if(this.state == STATE.RESET_ARM){
                this.grasp.home(true);
                grasp.armWait(); //Go to home position
                this.state = STATE.WAIT;
             }
             else if(this.state == STATE.FINISHED)
             {
              //  System.out.println("Disabling arm.");
                ohmm.driveSetVW((float)0.0, (float)0.0);
             

               //Move grasper to upright position for travel
              this.grasp.interpolateArm((float)(-250.0), (float)(100.0));
              this.grasp.armWait();
             
              //  ohmm.armEnable(false); //Disable arm
                break; //End the run loop
             }
      

           }
           catch(Exception e)
           {
             System.err.println("Exception in VisualServo thread.  ");
             ohmm.driveSetVW((float)0.0, (float)0.0);
             ohmm.armEnable(false);
             e.printStackTrace();
            // System.exit(1);
           } 
       }
    }

}
