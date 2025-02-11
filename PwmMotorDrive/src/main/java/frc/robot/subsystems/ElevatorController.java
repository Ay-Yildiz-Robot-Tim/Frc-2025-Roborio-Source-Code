package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import frc.robot.Constants;
public class ElevatorController {
    double Kp = 0.1, Ki = 0, Kd = 0;
    private PIDController elevatorPid = new PIDController(Kp, Ki, Kd);

    private PWMVictorSPX elevatorMotor = new PWMVictorSPX(Constants.PwmChannelContants.elvatorMotorsChannel);
    
    private double GetPidCalculate(int targetPoint, int point){
        return elevatorPid.calculate(point, targetPoint);
    }


    public void SetMotorSpeed(int targetPoint, int point){
        double speed = GetPidCalculate(targetPoint, point);
        elevatorMotor.set(speed);
    }
    
}
