package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class PneumaticSystem {
    private final Compressor compressor = new Compressor(0, PneumaticsModuleType.REVPH);
    boolean valfStateBool = false;

    // PCM'deki solenoid valf
    private final Solenoid solenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);

    public void SystemInıt(){
        compressor.disable(); // PCM’in otomatik kontrolünü devre dışı bırak
        solenoid.set(false);
    }

    public void SystemFrover(boolean valfOn, boolean valfOff){
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
        if(state){
            solenoid.set(true);
        }
        else{
            solenoid.set(false);
        }
    }
}
