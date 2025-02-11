# Robot Projesi

Bu proje, FRC robotumuzun çeşitli alt sistemlerini (elevator, platform hareketi, pnömatik sistem ve robot kolu) yönetmek için geliştirilmiştir. Robotumuzun hareket kontrolü, PID denetleyicileri ve joystick girişleri kullanılarak gerçekleştirilmiştir.

## Alt Sistemler

### 1. Elevator Controller
Elevator sistemi, robotun belirli bir yüksekliğe çıkmasını ve sabit kalmasını sağlamak için bir **PID kontrolcüsü** kullanır. PID kontrolü, motorun hedeflenen noktaya ulaşmasını ve dalgalanma olmadan stabil kalmasını sağlar.

#### Kullanılan Bileşenler:
- **PWMVictorSPX** motor sürücüsü
- **PIDController** (Kp = 0.1, Ki = 0, Kd = 0)
- Encoder ile yükseklik geri bildirimi

#### Çalışma Prensibi:
- `SetMotorSpeed(int targetPoint, int point)`: PID hesaplamalarını yaparak motor hızını belirler.
- `GetPidCalculate(int targetPoint, int point)`: Belirlenen hedef yüksekliğe ulaşmak için PID hesaplamasını yapar.

### 2. Platform Movement
Platform, robotun hareket etmesini sağlayan ana sistemdir. Dört adet **PWMVictorSPX** motoru kullanılarak yönlendirilir. Hareket için joystick girişleri kullanılır.

#### Özellikler:
- **Tank dönüşü** ve **doğrusal hareket** hesaplamaları
- **Hedef açı ve mesafeye** göre hız hesaplaması
- Joystick üzerinden manuel kontrol

#### Çalışma Prensibi:
- `PowerCalc(double powerFront, double powerBack, double axisX)`: İleri, geri ve dönüş hareketlerini hesaplar.
- `MoventManual(double powerFront, double powerBack, double axisX)`: Joystick ile manuel kontrol sağlar.
- `MoventAuto(double targetAngle, double distance)`: Otonom hareket için motor hızlarını ayarlar.

### 3. Pnömatik Sistem (Pneumatic System)
Robotun pnömatik sistemini yönetir. Basınçlı hava kullanarak valfleri açıp kapamaya yarar.

#### Kullanılan Bileşenler:
- **Compressor** (REV PH)
- **Solenoid valf**

#### Çalışma Prensibi:
- `SystemInıt()`: Pnömatik sistem başlatılır ve varsayılan kapalı moda alınır.
- `SystemFrover(boolean valfOn, boolean valfOff)`: Valf kontrolü yapılarak pnömatik sistem çalıştırılır.

### 4. Robot Arm Controller
Robot kolu, PID kontrolcüsü kullanılarak belirli açılara getirilir. Encoder verileri kullanılarak hassas konumlandırma yapılır.

#### Kullanılan Bileşenler:
- **PWMVictorSPX** motor sürücüsü
- **PIDController** (Kp = 0.07, Ki = 0, Kd = 0.001)
- Encoder ile açı geri bildirimi

#### Çalışma Prensibi:
- `SetMotorSpeed(int targetPoint, int point)`: PID hesaplamaları ile kol motorunun hızını belirler.
- `GetPidCalculate(int targetPoint, int point)`: Belirlenen hedef açıya ulaşmak için PID hesaplamasını yapar.

## Robot.java
Robotun ana kontrol dosyasıdır. Joystick girişlerini alarak ilgili alt sistemlere komutlar gönderir.

#### İşlevler:
- **Joystick verilerini okuma**: İleri, geri, dönüş, kol ve valf kontrolleri.
- **PID kontrolcülerini besleme**: Elevator ve kol sistemleri için hedef noktaları ayarlar.
- **Encoder verilerini kaydetme**: Robotun mevcut konum bilgilerini saklar.

#### Önemli Metodlar:
- `teleopPeriodic()`: Teleoperasyon modunda joystick girişlerini okuyarak motor ve sistemleri kontrol eder.
- `setSetPoint(int point)`: Elevator için hedef yüksekliği belirler.
- `setSetPointArm(int point)`: Robot kolu için hedef açıyı belirler.
- `saveEncoderData(int elevatorPulse, int armPulse)`: Encoder verilerini saklar.

## PID Kullanımı
Bu projede PID kontrolü, **hem belirtilen yüksekliğe ulaşmak hem de konumu sabit tutmak için** kullanılır. PID algoritması, encoder verileriyle sürekli karşılaştırma yaparak motor hızını dinamik olarak ayarlar.

### PID Faydaları:
- **Stabil Hareket:** Ani hız değişimlerini engelleyerek sarsıntısız bir hareket sağlar.
- **Hassas Konumlandırma:** Robot kolu ve elevator gibi sistemlerin doğru noktada kalmasını sağlar.
- **Uyarlanabilirlik:** Farklı ağırlıklara ve dış etkilere göre otomatik olarak ayarlanabilir.

## Kurulum ve Kullanım
1. **Kodları yükleyin**: WPILib kullanarak FRC robotuna yükleyin.
2. **Joystick bağlayın**: Robot kontrol sistemine bir joystick bağlayın.
3. **Teleoperasyon moduna geçin**: `teleopPeriodic()` metodunun joystick verilerini okuyarak motorları kontrol ettiğini görebilirsiniz.
4. **PID ayarlarını test edin**: Farklı Kp, Ki, Kd değerleriyle PID performansını test edin.

---

Bu proje FRC robot yarışmalarında otonom ve teleoperasyon modlarında stabil ve hassas hareket sağlamak için geliştirilmiştir.

