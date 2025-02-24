package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalOutput;

public class PneumaticSystem {
    DigitalOutput leftSuction = new DigitalOutput(4); // DIO 1'e bağlı LED
    DigitalOutput rightSuction = new DigitalOutput(5); // DIO 1'e bağlı LED

    boolean valfStateBool = false;
    
    public void SystemFrover(boolean valfOn, boolean valfOff){
        if(valfOff){
            valfStateBool = false;
        }
        else if(valfOn){
            valfStateBool = true;
        }

        ValfState(valfStateBool);
    }



    private void ValfState(boolean state){
        if(state){
            leftSuction.set(true);
            rightSuction.set(true);

        }
        else{
            leftSuction.set(false);
            rightSuction.set(false);
        }
    }
}
