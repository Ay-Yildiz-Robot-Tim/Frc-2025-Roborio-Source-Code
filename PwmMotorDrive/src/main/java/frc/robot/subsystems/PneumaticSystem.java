package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;

public class PneumaticSystem {
    private PWMVictorSPX valfMotor = new PWMVictorSPX(6);

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
            valfMotor.set(1);
        }
        else{
            valfMotor.set(0);
        }
    }
}
