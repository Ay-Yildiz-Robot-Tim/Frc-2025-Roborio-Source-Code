package frc.robot.subsystems;

public class PlatformMovement {
    
    public double[] PowerCalc(double powerX, double powerY) {//index 0 right motors index 1 left motors
        double leftPowerMotors = powerY;
        double rightPowerMotors = powerY;

        if(powerY < -0.1){//tank dönüşü için değer aralığı olması için bir aralık berlirleme
            leftPowerMotors = -powerY;//motorların ters bağlandığı ve benim joyistickden gelen değerimin ileri için 0 ve -1 arasında bir değer aldığından olayı ileri gitmesi sol motorlara pozitif değer gitmesi için - ile çarparak negatif değer veriyorum

        }
        else if(powerY > 0.1){
            leftPowerMotors = -powerY;//sağ motor ters bağlandığı için geri gitmesi için negatid değer almalı
        }

        else{
            if(powerX < -0.1){//tank dönüşünde sola gidiyorsa
                rightPowerMotors = powerX;// sağ motor ters - de ileri gider + değerde geri bu koşulda motor her halükarda ileri gidecek
                leftPowerMotors = powerX;//sol motor - de geri gideceği için bu koşulda motor geriye gidecektir
            }
            else if(powerX > 0.1){//tank dönüşünde sağa gidiyorsa
                rightPowerMotors = powerX;
                leftPowerMotors = powerX;
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
