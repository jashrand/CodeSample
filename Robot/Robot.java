import ohmm.*;
import static java.lang.Math.*;
import java.util.*;
import java.awt.Color;

public class Robot extends OHMMDrive{
    public static final float ROBOT_WIDTH = -108f;
    public static final float ROBOT_FRONT = 45f;
    public static final float ROBOT_BACK = -135f;
    
    public Point frontIR;
    public Point backIR;

    public float frontIRr;
    public float backIRr;
    
    public static List<List<Point>> rep = new ArrayList<List<Point>>();
    public static List<Point> frontIRs = new ArrayList<Point>();
    public static List<Point> backIRs = new ArrayList<Point>();
    
    static{
		List<Point> chassis = new ArrayList<Point>();
		List<Point> rightWheel = new ArrayList<Point>();
		List<Point> leftWheel = new ArrayList<Point>();

		chassis.add(new Point(50f,-108f));
		chassis.add(new Point(-150f,-108f));
		chassis.add(new Point(-150f,108f));
		chassis.add(new Point(50f,108f));
		rightWheel.add(new Point(40f,-108f));
		rightWheel.add(new Point(-40f,-108f));
		rightWheel.add(new Point(-40f,-98f));
		rightWheel.add(new Point(40f,-98f));
		leftWheel.add(new Point(40f,108f));
		leftWheel.add(new Point(-40f,108f));
		leftWheel.add(new Point(-40f,98f));
		leftWheel.add(new Point(40f,98f));

		rep.add(chassis);
		rep.add(rightWheel);
		rep.add(leftWheel);
    }

    public AllState currentState;

    public Robot(String arg){
		super(arg);
		//initialize sensors
		//this.driveResetPose();
		this.senseConfigDigital(30,true,false);
		this.senseConfigDigital(31,true,false);
		this.senseConfigAnalogIR(2,1);
		this.senseConfigAnalogIR(3,1);
		this.motSetVelCmd(0f,0f);
		currentState = this.allState();
    }

    public void pollRobot(){
		currentState = this.allState(currentState);
		
		//pull ir sensor data
		frontIRr = currentState.sense.analog[0];
		backIRr = currentState.sense.analog[1];

		//convert to robot frame
		//is (frontIR + 108f, 50f) 
		//and (backIR + 108f, -60f)

		//convert to world frame
			float[] pose = this.driveGetPose();
		frontIR = convertToWorld(ROBOT_FRONT, -frontIRr+ROBOT_WIDTH,
					 pose[0], //x
						 pose[1], //y
						 pose[2]); //theta

		backIR = convertToWorld (ROBOT_BACK, -backIRr+ROBOT_WIDTH,
					pose[0],
					pose[1],
						pose[2]);
		
		//store for posterity
		if(frontIRr < 400)
			frontIRs.add(frontIR);
		if(backIRr < 400)
			backIRs.add(backIR);
	}

    public AllState getCurrentState(){
		return currentState;
    }
    
    public Point convertToWorld(float xr, float yr, 
				float robotX, float robotY,float theta ){
		float xw, yw;

		xw = ((float)cos(theta))*xr - ((float)sin(theta))*yr;
		yw = ((float)sin(theta))*xr + ((float)cos(theta))*yr;

		xw += robotX;
		yw += robotY;
		return new Point(xw,yw);
    }



    //Override driveTurn to account for rug friction
    @Override
    public boolean driveTurn(float theta){
       return super.driveTurn(theta * Constants.RUG_TURN_FRICTION);
    }


    /**
     * Given a direction, turn to it
     **/
    public void turnToTheta(float desiredTheta){
        float curTheta = this.driveGetPose()[2];
		float dTheta = desiredTheta - curTheta;
		if(dTheta > Math.PI){
			dTheta = dTheta - (float)(2*Math.PI);
		}
		else if(dTheta < -Math.PI){
			dTheta = dTheta + (float)(2*Math.PI);
		}
		this.driveTurn(dTheta * Constants.RUG_TURN_FRICTION);
    }	    
}

