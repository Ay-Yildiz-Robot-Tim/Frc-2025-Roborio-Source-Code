package frc.robot.subsystems;

import javax.security.sasl.AuthorizeCallback;

import com.fasterxml.jackson.databind.cfg.EnumFeature;

import edu.wpi.first.wpilibj.PS4Controller.Axis;

public class PlatformMovement {
    
    // Motorun maksimum RPM'si
    private static final int MAX_RPM = 5310;  // Maksimum RPM
    private static final int MIN_RPM = -5310; // Minimum RPM (geri yön)

    //ileri gidiş için
    private double[] FrontLineer(double powerFront, double axisX){
        //default atanması gereken değer
        double leftPowerMotors = powerFront;
        double rightPowerMotors = -powerFront;

        //açısal dönüş koşulları
        if(axisX > .1){
            leftPowerMotors = powerFront * axisX;
            rightPowerMotors = -powerFront;
        }
        else if(axisX < -.1){
            leftPowerMotors = powerFront;
            rightPowerMotors = -powerFront * axisX;
        }

        return new double[]{rightPowerMotors, leftPowerMotors};
    }

    //geri gidiş hesaplama
    private double[] BackLineer(double powerBack, double axisX){
        double leftPowerMotors = -powerBack;
        double rightPowerMotors = powerBack;

        //açısal dönüş koşulları ve hesaplanması
        if(axisX > .1){
            leftPowerMotors = -powerBack;
            rightPowerMotors = powerBack * axisX;
        }
        else if(axisX < -.1){
            leftPowerMotors = -powerBack * axisX;
            rightPowerMotors = powerBack;
        }

        return new double[]{rightPowerMotors, leftPowerMotors};
    }

    //tank dönüşü güç hesaplması
    private double[] TankTurn(double axisX){
        /*
        //sol dönüş ayarlamsı sağ motorlar ters bağlandığı için sağ motorlara - vermek ileri hareketi sağla
        if(axisX > .1){
            leftPowerMotors = -axisX;
            rightPowerMotors = -axisX;
        }
        //sağ dönüş ayarlaması için sağ motorlar + da geri gidecek
        else if(axisX < -.1){
            leftPowerMotors = -axisX;
            rightPowerMotors = -axisX;
        }
        return new double[]{rightPowerMotors, leftPowerMotors};
        */

        //tank dönüşü güç hesaplması
        double leftPowerMotors = 0;
        double rightPowerMotors = 0;

        if(axisX < -.1 || axisX > .1){
            leftPowerMotors = -axisX;
            rightPowerMotors = -axisX;
        }
        return new double[] {rightPowerMotors, leftPowerMotors};
    }

    public double[] PowerCalc(double powerFront, double powerBack, double axisX) {//index 0 right motors index 1 left motors
        //ileri buttonunda hareket varmı
        if(powerFront > .1){
            return FrontLineer(powerFront, axisX);
        }

        //geri butonunda hareket varmı
        else if(powerBack > .1){
            return BackLineer(powerBack, axisX);
        }

        //yön butonunda hareket varmı
        else{
            return TankTurn(axisX);
        }
    }

    // Dönüş için gereken motor gücü hesapla
    private double calculateTurnPower(double targetAngle) {
        // 20 dereceyi baz alıyoruz, diğer açılar için normalize ediyoruz
        double turnPower = targetAngle / 20.0;  // 20 dereceyi baz alıyoruz
        return turnPower;
    }

    // Mesafe için gereken motor gücü hesapla
    private double calculateDistancePower(double distance) {
        // Dinamik mesafe: Mesafe parametre olarak değişebilir
        // Burada mesafeyi 1.2 metre (120 cm) baz alarak normalize ediyoruz
        double distancePower = distance / 100.0;  // Mesafe parametre olarak dinamik olacak
        return distancePower;
    }
    
    // Robotun dönüş ve mesafe için gereken motor gücünü hesapla
    public double[] PowerCalc(double targetAngle, double distance) {
        double leftPowerMotors = calculateDistancePower(distance);  // Başlangıçta sol motor gücü
        double rightPowerMotors = calculateDistancePower(distance); // Başlangıçta sağ motor gücü

        // Eğer açı -5 ile +5 arasında ise düz gitsin
        if (targetAngle >= -5 && targetAngle <= 5) {
            // Mesafe için güç sadece bir defa hesaplanır
            leftPowerMotors = calculateDistancePower(distance);
            rightPowerMotors = calculateDistancePower(distance);
        } else {
            // Dönüş için motor gücünü hesapla
            double turnPower = calculateTurnPower(targetAngle);

            // Dönüş yönüne göre motor güçlerini ayarla
            if (targetAngle > 0) { // Sağ dönüş
                rightPowerMotors = calculateDistancePower(distance) * (1 + turnPower); // Sağ motor hızı (1 = yüksek hız)
                leftPowerMotors = calculateDistancePower(distance) * (1 - turnPower);  // Sol motor hızı (0 = düşük hız)
            } else { // Sol dönüş
                rightPowerMotors = calculateDistancePower(distance) * (1 - turnPower);  // Sağ motor hızı (0 = düşük hız)
                leftPowerMotors = calculateDistancePower(distance) * (1 + turnPower);   // Sol motor hızı (1 = yüksek hız)
            }
        }

        // Motor güçlerini 0 ile 1 arasında sınırla
        rightPowerMotors = Math.max(0, Math.min(1, rightPowerMotors)); // Sağ motor için güç
        leftPowerMotors = Math.max(0, Math.min(1, leftPowerMotors));   // Sol motor için güç

        // Sağ ve sol motor güçlerini döndür
        return new double[] {-rightPowerMotors, leftPowerMotors};
    }
        
}
