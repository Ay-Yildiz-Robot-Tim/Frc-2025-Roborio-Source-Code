package frc.robot.subsystems;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import frc.robot.Constants;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;

public class RobotArmController {
    double Kp = 0.07, Ki = 0, Kd = 0.001;
    private PIDController elevatorPid = new PIDController(Kp, Ki, Kd);
    private PWMVictorSPX armMotor = new PWMVictorSPX(Constants.PwmChannelContants.armMotorChannel);

    private double GetPidCalculate(int targetPoint, int point){
        return elevatorPid.calculate(point, targetPoint);
    }
    
    public void SetMotorSpeed(int targetPoint, int point){
        double speed = GetPidCalculate(targetPoint, point);
        speed *= 0.25; // Hızı %50 düşürüyoruz  
        armMotor.set(speed);
    }
}
    
