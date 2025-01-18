package frc.robot.subsystems;

import com.fasterxml.jackson.databind.cfg.EnumFeature;

public class PlatformMovement {
    
    public double[] PowerCalc(double powerFront, double powerBack, double axisX) {//index 0 right motors index 1 left motors
        double leftPowerMotors = 0;
        double rightPowerMotors = 0;
        
        //rt buttonundan gelen hızın ileri olarak atanması
        if(powerFront > .1){
            //default atanması gereken değer
            leftPowerMotors = powerFront;
            rightPowerMotors = -powerFront;

            //açısal dönüş koşulları
            if(axisX > .1){
                leftPowerMotors = powerFront * axisX;
                rightPowerMotors = -powerFront;
            }
            else if(axisX < -.1){
                leftPowerMotors = powerFront;
                rightPowerMotors = -powerFront * axisX;
            }
        }
        else if(powerBack > .1){
            leftPowerMotors = -powerBack;
            rightPowerMotors = powerBack;

            if(axisX > .1){
                leftPowerMotors = -powerBack;
                rightPowerMotors = powerBack * axisX;
            }
            else if(axisX < -.1){
                leftPowerMotors = -powerBack * axisX;
                rightPowerMotors = powerBack;
            }
        }
        else{
            if(axisX > .1){
                leftPowerMotors = -axisX;
                rightPowerMotors = -axisX;
            }
            else if(axisX < -.1){
                leftPowerMotors = -axisX;
                rightPowerMotors = -axisX;
            }
        }

        return new double[]{rightPowerMotors, leftPowerMotors};
    }

    public double[] PowerCalc(int targetAngle, int currentRPM, int targetRPM, int currentAngle) {
        double[] motorPowers = {0.0, 0.0};
    
        // Kazanç Sabitleri (test ile ayarlanabilir)
        double kP_angle = 0.02; // Açı için orantısal kazanç
        double kI_angle = 0.001; // Açı için integral kazanç
        double kP_rpm = 0.01;   // RPM için orantısal kazanç
        double kI_rpm = 0.0005; // RPM için integral kazanç
    
        // Hata ve integral terimleri
        double angleError = targetAngle - currentAngle;
        double rpmError = targetRPM - currentRPM;
    
        // Integral terimlerini takip için static tanımlayın (hataların toplamını tutmak için)
        double angleIntegral = 0.0;
        double rpmIntegral = 0.0;
    
        // Hataların toplamı (integral terimi)
        angleIntegral += angleError;
        rpmIntegral += rpmError;
    
        // Düzeltme değerleri
        double angleCorrection = kP_angle * angleError + kI_angle * angleIntegral;
        double speedCorrection = kP_rpm * rpmError + kI_rpm * rpmIntegral;
    
        // Motor güçlerini hesapla
        motorPowers[0] = speedCorrection + angleCorrection; // Sağ motor
        motorPowers[1] = speedCorrection - angleCorrection; // Sol motor
    
        // Normalize sınırlandırma (-1.0 ile 1.0 arasında)
        motorPowers[0] = Math.max(-1.0, Math.min(1.0, motorPowers[0]));
        motorPowers[1] = Math.max(-1.0, Math.min(1.0, motorPowers[1]));
    
        return motorPowers;
    }
        
}
