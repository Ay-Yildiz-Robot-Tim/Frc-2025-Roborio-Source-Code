package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class PneumaticSystem {
    private final Compressor compressor = new Compressor(0, PneumaticsModuleType.REVPH);

    // PCM'deki solenoid valf
    private final Solenoid solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);

    public void SystemInıt(){
        compressor.disable(); // PCM’in otomatik kontrolünü devre dışı bırak
        solenoid.set(false);
    }

    public void SystemFrover(boolean valfOn, boolean valfOff){
        boolean valfStateBool = false;
        double oldPressure = compressor.getPressure();
        if(valfOff){
            valfStateBool = false;
        }
        else if(valfOn){
            valfStateBool = true;
        }

        compressor.enableAnalog(60, 90);
        ValfState(valfStateBool, oldPressure);
    }
    
    private void ValfState(boolean state, double oldPressure){
        double newPressure = compressor.getPressure();
        if(state){
            while(oldPressure - newPressure < 3){
                solenoid.set(true);
                newPressure = compressor.getPressure();
            }
        }
        else{
            solenoid.set(false);
        }
    }

}
