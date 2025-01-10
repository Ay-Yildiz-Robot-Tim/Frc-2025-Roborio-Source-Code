Robot Project
Bu proje, FIRST Robotics Competition (FRC) için geliştirilmiş bir robotun temel kontrol sistemini içermektedir. Robotun hareketini, joystick üzerinden verilen girişler ile kontrol etmek için kullanılan basit bir kod yapısını sunar. WPILib kütüphanesi kullanılarak, robotun motorları kontrol edilmekte ve joystick verileri alınarak motor hızları hesaplanmaktadır.

Proje Yapısı
Robot.java
Bu dosya, robotun tüm fonksiyonel işlevlerini ve davranışlarını yöneten ana sınıftır. Bu sınıf, robotun başlatılmasından, hareket etmesine kadar tüm süreçleri yönetir.

Önemli Fonksiyonlar:
robotInit(): Robot başlatıldığında yapılan ilk işlemleri içerir. Motorlar ve joystick burada başlatılır.
teleopInit(): Teleoperated (uzaktan kontrol) moduna geçildiğinde yapılan işlemleri içerir. Joystick verileri alınır ve motor hızları hesaplanır.
teleopPeriodic(): Teleoperated modunda sürekli olarak çalıştırılan fonksiyondur. Burada joystick verilerine göre motorlar çalıştırılır.
Motor Kontrolü:
PWMVictorSPX motor denetleyicileri kullanılarak robotun her iki tarafındaki motorlar (leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor) kontrol edilir.
Joystick'in X ve Y eksenlerindeki değerler alınarak, robotun yönü ve hızı belirlenir.
Constants.java
Bu dosya, robotun tüm sabitlerini barındırır. Motorların PWM kanal numaraları ve joystick'in port numarası burada tanımlanır.

Sabitler:
OperatorConstants: Operatör (sürücü) joystick'inin bağlandığı portu tanımlar.
PwmChannelContants: Motorların bağlandığı PWM portlarını tanımlar.
Kullanım
Gerekli Kütüphaneler
WPILib: FRC robotları için geliştirilen kütüphaneleri içerir.
Motor kontrolü için PWMVictorSPX sınıfı kullanılır.
Joystick sınıfı ile joystick verileri alınır.
CommandScheduler sınıfı ile robotun komutları yönetilir.
Bağlantılar
Joystick: Robotun hareketini kontrol etmek için bir joystick kullanılır. Bu joystick, robotun hareketine yön vermek için X ve Y eksenleri kullanılarak hareket kontrol edilir.
Motorlar: Motorlar, joystick verilerine göre hızlandırılır veya yavaşlatılır.
Motor Pin Bağlantıları
leftFrontMotor: PWM portu 0
leftBackMotor: PWM portu 1
rightFrontMotor: PWM portu 2
rightBackMotor: PWM portu 3
Joystick Bağlantısı
Joystick, 0. port'a bağlanmıştır. Bu, robotun hareketini yönlendirmek için kullanılan ana giriş cihazıdır.

Nasıl Çalışır?
Başlatma: robotInit() fonksiyonu robot başlatıldığında çalıştırılır, burada motorlar ve joystick başlatılır.
Teleoperated Mod: teleopInit() fonksiyonu ile joystick'ten gelen veriler alınır, ardından motor hızları hesaplanarak robotun hareketi sağlanır.
Motor Hızları: Joystick'in Y eksenindeki hareket robotun ileri-geri hareketini, X eksenindeki hareket ise dönüş hareketini kontrol eder. İleri hareket için Y değeri doğrudan motor hızlarına aktarılır. Dönüş içinse Y ve X değerlerinin kombinasyonu kullanılarak dönüş hızları hesaplanır.
Lisans
Bu yazılım, WPILib BSD lisansı altında dağıtılmaktadır. Daha fazla bilgi için, proje kök dizininde bulunan LICENSE dosyasını inceleyebilirsiniz.

