//This class is used to control the OHMM Arm


import ohmm.*;
import ohmm.OHMM.*;
import java.lang.Exception;

public class ArmControl extends Grasp
{

  //Constructor
  public ArmControl(OHMM ohmm) throws Exception {

        super((OHMMDrive)ohmm);

   }


  //Initialize arm control
  public void init()
  {
      super.init();
      this.cal();
      this.home(false);
  }



  //Move arm to the given x, y orientation (in robot frame)
    public void setArmPos(float x, float z)
    {
       armWait();
       dumpArm();
    }
	
	// for dancing
	public void goCrazy() {
		this.home(true);
	}

  //Go to the carrying pose (Arm straight up)
  public void carryPose(){
      this.stow(true);
      armWait();
  }

  //Put a target down again
  public void placeSequence(){

		System.out.println("home pose");
		this.home(true);
		armWait(); //Go to home position
		System.out.println("interpolate arm");

		interpolateArm((float)0.0, (float)(-100.0)); //Move 100 up in z
		armWait();
		System.out.println("arm setgriper");

		ohmm.armSetGripper((float)1.0); 
			armWait();
                ohmm.driveStraight(-100.0f); //Back up
                GlobalNavigation.blockForQueue((Robot)ohmm);
                carryPose();
                closeGripper();
                
  }

public void closeGripper(){
	ohmm.armSetGripper((float)0.0);
	armWait();
}


  
}
