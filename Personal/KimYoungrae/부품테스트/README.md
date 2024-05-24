# 지급받은 키트 검수

텍스트 파일은 매번 확인하기 귀찮아서 작성함.

## 내가 진행한 목차
- 조이스틱 모듈 Arduino Joystick Shield
- 적외선 센서와 리모컨 CHQ1838
- 비접촉식 적외선 온도센서 gy-906
- RTC모듈(tick) DS1302
- bluetooth 모듈 HC06
- WIFI 모듈 ESP8266
- USB 마이크 USB Mini Microphone For Raspberry PI
- 스피커 모듈 lm386

<br>

## 조이스틱 실드

```

[소스코드]
const int buttonPin2 = 2;                           // 조이스틱 쉴드의 버튼이 누르는 걸 입력받기 위해 선언
const int buttonPin3 = 3;
const int buttonPin4 = 4;
const int buttonPin5 = 5;
const int buttonPin6 = 6; // E
const int buttonPin7 = 7; // F
 
void setup() {
 
  Serial.begin(9600);                               // 시리얼 통신을 시작하며, 통신속도는 9600
 
  pinMode(buttonPin2, INPUT_PULLUP );
  pinMode(buttonPin3, INPUT_PULLUP );
  pinMode(buttonPin4, INPUT_PULLUP );
  pinMode(buttonPin5, INPUT_PULLUP );
  pinMode(buttonPin6, INPUT_PULLUP );
  pinMode(buttonPin7, INPUT_PULLUP );
  
}
 
void loop() {
 
 
  int X = analogRead(0);                           // 변수 X에 아날로그 0번핀에 입력되는 신호를 대입
  int Y = analogRead(1);                           // 변수 Y에 아날로그 1번핀에 입력되는 신호를 대입
 
  int buttonValue2 = digitalRead(2);               // buttonValue값 선언
  int buttonValue3 = digitalRead(3);
  int buttonValue4 = digitalRead(4);
  int buttonValue5 = digitalRead(5);
 
  int buttonValue6 = digitalRead(6);
  int buttonValue7 = digitalRead(7);
 
 
  Serial.print("joy stick  ");                       // 조이스틱 x값, y값 시리얼모니터에 출력
  Serial.print("X");
  Serial.print(":");
  Serial.print(X);
  Serial.print("  ");
  Serial.print("Y");
  Serial.print(":");
  Serial.println(Y);
 
  if (buttonValue2 == LOW) {                       // if문을 이용하여 각 버튼이 눌리면 알파벳이 시리얼모니터에 출력되도록 설정
    Serial.print("joy stick  ");
    Serial.print("X");
    Serial.print(":");
    Serial.print(X);
    Serial.print("  ");
    Serial.print("Y");
    Serial.print(":");
    Serial.print(Y);
    Serial.print("   |");
    Serial.println("A pushed!");
  }
  if (buttonValue3 == LOW) {
    Serial.print("joy stick  ");
    Serial.print("X");
    Serial.print(":");
    Serial.print(X);
    Serial.print("  ");
    Serial.print("Y");
    Serial.print(":");
    Serial.print(Y);
    Serial.print("   |");
    Serial.println("B pushed!");
  }
  if (buttonValue4 == LOW) {
    Serial.print("joy stick  ");
    Serial.print("X");
    Serial.print(":");
    Serial.print(X);
    Serial.print("  ");
    Serial.print("Y");
    Serial.print(":");
    Serial.print(Y);
    Serial.print("   |");
    Serial.println("C pushed!");
  }
  if (buttonValue5 == LOW) {
    Serial.print("joy stick  ");
    Serial.print("X");
    Serial.print(":");
    Serial.print(X);
    Serial.print("  ");
    Serial.print("Y");
    Serial.print(":");
    Serial.print(Y);
    Serial.print("   |");
    Serial.println("D pushed!");
  }
  if (buttonValue6 == LOW) {
    Serial.print("joy stick  ");
    Serial.print("X");
    Serial.print(":");
    Serial.print(X);
    Serial.print("  ");
    Serial.print("Y");
    Serial.print(":");
    Serial.print(Y);
    Serial.print("   |");
    Serial.println("E pushed!");
  }
  if (buttonValue7 == LOW) {
    Serial.print("joy stick  ");
    Serial.print("X");
    Serial.print(":");
    Serial.print(X);
    Serial.print("  ");
    Serial.print("Y");
    Serial.print(":");
    Serial.print(Y);
    Serial.print("   |");
    Serial.println("F pushed!");
  }
 
  delay(500);                                        // 0.5초동안 지속
}


[결과]
joy stick  X:498  Y:503   |A pushed!
joy stick  X:499  Y:504
joy stick  X:498  Y:504
joy stick  X:499  Y:504
joy stick  X:499  Y:504
joy stick  X:498  Y:504
joy stick  X:498  Y:504   |B pushed!
joy stick  X:498  Y:504
joy stick  X:498  Y:504
joy stick  X:498  Y:503
joy stick  X:498  Y:503   |C pushed!
joy stick  X:499  Y:504
joy stick  X:498  Y:504
joy stick  X:498  Y:503
joy stick  X:498  Y:503   |D pushed!
joy stick  X:498  Y:504
joy stick  X:499  Y:1022
joy stick  X:499  Y:1022
joy stick  X:499  Y:503
joy stick  X:499  Y:0
joy stick  X:498  Y:0
joy stick  X:498  Y:504
joy stick  X:1023  Y:504
joy stick  X:1023  Y:504
joy stick  X:1022  Y:504
joy stick  X:0  Y:504
joy stick  X:0  Y:503
joy stick  X:498  Y:504
joy stick  X:499  Y:504
joy stick  X:498  Y:504
joy stick  X:499  Y:504
joy stick  X:498  Y:504
joy stick  X:498  Y:503
joy stick  X:498  Y:503   |E pushed!
joy stick  X:499  Y:504
joy stick  X:498  Y:504
joy stick  X:498  Y:503
joy stick  X:498  Y:503   |F pushed!

```


<br>

## 적외선 센서

```

아두이노 리모컨

IRremote라이브러리 설치 후 
SimpleReceiver 예제 사용


[결과]
Protocol=NEC Address=0x0 Command=0x42 Repeat gap=38950us

Protocol=NEC Address=0x0 Command=0x52 Raw-Data=0xAD52FF00 32 bits LSB first
Send with: IrSender.sendNEC(0x0, 0x52, <numberOfRepeats>);

Protocol=NEC Address=0x0 Command=0x52 Repeat gap=38950us

Protocol=NEC Address=0x0 Command=0x4A Raw-Data=0xB54AFF00 32 bits LSB first
Send with: IrSender.sendNEC(0x0, 0x4A, <numberOfRepeats>);

Protocol=NEC Address=0x0 Command=0x4A Repeat gap=38950us

Protocol=NEC Address=0x0 Command=0x42 Raw-Data=0xBD42FF00 32 bits LSB first
Send with: IrSender.sendNEC(0x0, 0x42, <numberOfRepeats>);

Protocol=NEC Address=0x0 Command=0x42 Repeat gap=38950us

Protocol=NEC Address=0x0 Command=0x52 Raw-Data=0xAD52FF00 32 bits LSB first
Send with: IrSender.sendNEC(0x0, 0x52, <numberOfRepeats>);

Protocol=NEC Address=0x0 Command=0x52 Repeat gap=38950us

Protocol=NEC Address=0x0 Command=0x4A Raw-Data=0xB54AFF00 32 bits LSB first
Send with: IrSender.sendNEC(0x0, 0x4A, <numberOfRepeats>);

Protocol=NEC Address=0x0 Command=0x4A Repeat gap=38950us

Protocol=NEC Address=0x0 Command=0x4A Repeat gap=94800us

```


<br>


## 비접촉식 적외선 온도센서

```

비접촉식 적외선 온도선서

[라이브러리] Adafruit MLX90614 Library 2.1.5 (dependency all download)
[예제] mlxtest
[핀] Vcc 5V / GND GND / SDA A4 / SCL A5


[결과]
// 대기
Ambient = 31.15*C	Object = 26.99*C
Ambient = 88.07*F	Object = 80.58*F

// 대기 -> 손
Ambient = 31.17*C	Object = 31.71*C
Ambient = 88.11*F	Object = 89.08*F

// 손
Ambient = 31.17*C	Object = 34.45*C
Ambient = 88.11*F	Object = 94.01*F


```


<br>

## RTC

```

RTC모듈 테스트
[핀] VCC 5V / GND GND / CLK 5 / DAT 6 / RST 7
[라이브러리] https://blog.naver.com/boilmint7/221911079454 에서 DS1032.zip 다운 후 IDE에서 .zip파일로 라이브러리 세팅 선택
[코드]
#include <DS1302.h>                             // DS1302 라이브러리 헥사 선언
const int CLK = 5;                                   // Clock 을  5번핀 설정
const int DAT = 6;                                   // Data를  6번핀 설정
const int RST = 7;                                   // Reset을  7번핀 설정
DS1302 myrtc(RST, DAT, CLK);              // DS1302  객체 설정
void setup() 
{
    myrtc.halt(false);                                   // 동작 모드로 설정
    myrtc.writeProtect(false);                      // 시간 변경을 가능하게 설정
    Serial.begin(9600);                                // 시리얼 통신 설정
    Serial.println("DS1302RTC Test");        // 시리얼 모니터에 ("   ") 내용을 출력
    Serial.println("---------------");                  // 시리얼 모니터에 ("   ") 내용을 출력
    myrtc.setDOW(MONDAY);                    // 요일 설정
    myrtc.setTime(14, 30, 20);                    // 시간 설정 ( 시간 , 분 , 초 )
    myrtc.setDate(13, 4, 2020);                  // 날짜 설정 ( 일 , 월 , 년도 )
}
void loop() 

{
    Serial.print(myrtc.getDOWStr());           // 시리얼 모니터에 요일 출력
    Serial.print(" ");
    Serial.print(myrtc.getDateStr());             // 시리얼 모니터에 날짜 출력
    Serial.print(" -- ");
    Serial.println(myrtc.getTimeStr());           // 시리얼 모니터에 시간 출력
    delay(1000);                                            // 1초의 딜레이

}





[결과]
---------------
Monday 13.04.2020 -- 14:30:20
Monday 13.04.2020 -- 14:30:21
Monday 13.04.2020 -- 14:30:22
Monday 13.04.2020 -- 14:30:23
Monday 13.04.2020 -- 14:30:24
Monday 13.04.2020 -- 14:30:25
Monday 13.04.2020 -- 14:30:26
Monday 13.04.2020 -- 14:30:27
Monday 13.04.2020 -- 14:30:28
Monday 13.04.2020 -- 14:30:29
Monday 13.04.2020 -- 14:30:30
Monday 13.04.2020 -- 14:30:31


```

<br>

## bluetooth

```

아두이노 HC-06 블루투스 모듈 AT명령어 확인

[핀] 모듈 아두이노 / VCC 5v / GND GND / TX 2 / RX 3
[코드]
#include <SoftwareSerial.h>                // Serial 통신을 하기 위해 선언
SoftwareSerial BTSerial(2, 3);             
void setup()
{
    Serial.begin(9600);                          

    BTSerial.begin(9600);                     
}

void loop() 
{
    if(BTSerial.available())                     // BTSerial에 입력이 되면

    Serial.write(BTSerial.read());           // BTSerial에 입력된 값을 시리얼 모니터에 출력

    if(Serial.available())                          // 시리얼 모니터에 입력이 되면

    BTSerial.write(Serial.read());           // 그 값을 BTSerial에 출력
}


[시리얼 모니터 설정]
Both NL & CR / 9600 baud



[결과] (AT를 입력으로 넣었을 경우 OK한 줄 이 나옴)
12:09:06.032 -> OK
12:09:14.487 -> OK
12:09:15.646 -> OK
12:09:16.360 -> OK
12:09:16.892 -> OK
12:09:17.436 -> OK
12:09:17.963 -> OK

```


<br>

## WIFI

```

아두이노 ESP8266 AT통신(UART잘 연결된건가) 확인

[핀] VCC 3.3v / GND GND / ESP측 RX 아두이노측 3(TX) / ESP측 TX 아두이노측 2(RX)

[코드] 
#include <SoftwareSerial.h>
SoftwareSerial mySerial(2,3); //RX, TX

void setup() {
   Serial.begin(9600);
   mySerial.begin(9600);
}

void loop() {
if(mySerial.available())
{
  Serial.write(mySerial.read());
}
if(Serial.available())
{
  mySerial.write(Serial.read());
}
}


시리얼 모니터에서 Both NL & CR / 9600 baud 설정

[결과]
11:48:01.405 -> AT
11:48:01.405 -> 
11:48:01.405 -> OK
11:57:24.327 -> AT
11:57:24.371 -> 
11:57:24.371 -> OK



```

<br>

## USB 마이크

```

[참고자료] http://www.makeshare.org/bbs/board.php?bo_table=raspberrypi&wr_id=76&sst=wr_hit&sod=asc&sop=and&page=5
USB 동글형 마이크 테스트

라즈베리파이를 이용함.


sudo vim /usr/share/alsa/alsa.conf 
에서 
defaults.ctl.card 0
defaults.pcm.card 0
을 
각 각
defaults.ctl.card 1
defaults.pcm.card 1
로 바꿈.

그 다음에 
sudo vim /usr/share/alsa/.asoundrc
를 한 뒤에
pcm.!default {
        type hw
        card 1
}

ctl.!default {
        type hw
        card 1
}

를 입력한 뒤에 저장을 한다.

이제 라즈베리파이를 통해 크롬에 접속한다.
다음 주소로 이동한다.

onlinemictest.com
권한 허용 팝업이 나오면 허가를 한다.

보이는 웹페이지에서 The mic test 밑에 있는 재생 버튼을 누른다.
그런 다음 동글에 대고 소리를 지른다.

다음과 같은 사진을 볼 수 있다.

```

<br>

## 스피커 모듈


```

#define NOTE_B0 31 #define NOTE_C1 33 #define NOTE_CS1 35 #define NOTE_D1 37 #define NOTE_DS1 39 #define NOTE_E1 41 #define NOTE_F1 44 #define NOTE_FS1 46 #define NOTE_G1 49 #define NOTE_GS1 52 #define NOTE_A1 55 #define NOTE_AS1 58 #define NOTE_B1 62 #define NOTE_C2 65 #define NOTE_CS2 69 #define NOTE_D2 73 #define NOTE_DS2 78 #define NOTE_E2 82 #define NOTE_F2 87 #define NOTE_FS2 93 #define NOTE_G2 98 #define NOTE_GS2 104 #define NOTE_A2 110 #define NOTE_AS2 117 #define NOTE_B2 123 #define NOTE_C3 131 #define NOTE_CS3 139 #define NOTE_D3 147 #define NOTE_DS3 156 #define NOTE_E3 165 #define NOTE_F3 175 #define NOTE_FS3 185 #define NOTE_G3 196 #define NOTE_GS3 208 #define NOTE_A3 220 #define NOTE_AS3 233 #define NOTE_B3 247 #define NOTE_C4 262 #define NOTE_CS4 277 #define NOTE_D4 294 #define NOTE_DS4 311 #define NOTE_E4 330 #define NOTE_F4 349 #define NOTE_FS4 370 #define NOTE_G4 392 #define NOTE_GS4 415 #define NOTE_A4 440 #define NOTE_AS4 466 #define NOTE_B4 494 #define NOTE_C5 523 #define NOTE_CS5 554 #define NOTE_D5 587 #define NOTE_DS5 622 #define NOTE_E5 659 #define NOTE_F5 698 #define NOTE_FS5 740 #define NOTE_G5 784 #define NOTE_GS5 831 #define NOTE_A5 880 #define NOTE_AS5 932 #define NOTE_B5 988 #define NOTE_C6 1047 #define NOTE_CS6 1109 #define NOTE_D6 1175 #define NOTE_DS6 1245 #define NOTE_E6 1319 #define NOTE_F6 1397 #define NOTE_FS6 1480 #define NOTE_G6 1568 #define NOTE_GS6 1661 #define NOTE_A6 1760 #define NOTE_AS6 1865 #define NOTE_B6 1976 #define NOTE_C7 2093 #define NOTE_CS7 2217 #define NOTE_D7 2349 #define NOTE_DS7 2489 #define NOTE_E7 2637 #define NOTE_F7 2794 #define NOTE_FS7 2960 #define NOTE_G7 3136 #define NOTE_GS7 3322 #define NOTE_A7 3520 #define NOTE_AS7 3729 #define NOTE_B7 3951 #define NOTE_C8 4186 #define NOTE_CS8 4435 #define NOTE_D8 4699 #define NOTE_DS8 4978


#include <pitches.h>
int speakerOut = 4; 

void setup() {
  pinMode(speakerOut, OUTPUT);
}
void loop() {
  tone(speakerOut, NOTE_C6, 1000);
  delay(1000); 
  tone(speakerOut, NOTE_D6, 1000);
  delay(1000);
  tone(speakerOut, NOTE_E6, 1000);
  delay(1000);
  tone(speakerOut, NOTE_F6, 1000);
  delay(1000);
  tone(speakerOut, NOTE_G6, 1000);
  delay(1000);
  tone(speakerOut, NOTE_A6, 1000);
  delay(1000);
  tone(speakerOut, NOTE_B6, 1000);
  delay(1000);
  tone(speakerOut, NOTE_C7, 1000);
  delay(1000);
  while(1);

}

```


<br>
