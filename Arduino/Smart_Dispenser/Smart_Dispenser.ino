
#include "U8glib.h"
#include "EEPROM.h"
#include "Encoder.h"  // https://github.com/PaulStoffregen/Encoder


// 핀 정의
#define encoderA 2  // pin of motor encoder - interrupt of arduino nano
#define encoderB 3  // pin of motor encoder - interrupt of arduino nano
#define up_Pin 7    // Up switch
#define sel_Pin 8   // Select switch
#define down_Pin 9  // Down switch
//#define limit_Pin 10  // Limit sensor

#define AIN_Pin_1 5  // drv8833 signal 1 : PWM
#define AIN_Pin_2 6  // drv8833 signal 2 : PWM

// 전역 변수
long newPosition;
long oldPosition;    // encoder position
boolean direction;   // true = cylinder advance, false = cylinder retraction
int min_error = 30;  // encoder difference less than this number will be ignored

long currentPos;  // cylinder first(=current) position
long targetPos;   // cylinder position to move
long difference;  // difference of currentPos and targetPos

boolean up_Flag = false, select_Flag = false, down_Flag = false;  // switch flag

byte address;  // EEPROM address (0:stage, 1:Extrusion, 2:Retraction, 3:Speed)
byte Stage = 0, previous_Stage = 0;  // 0 = manual, 1 = auto, 2 = PnP, 4 = Setup
byte setup_Stage = 0;        // setup stage, 0 = extrusion, 1 = retraction, 2 = speed
byte Extrusion;              // extrusion set amount.(min:0 / max:100), maximum extrusion moves the cylinder 5mm
byte Retraction;             // retraction set amount.(min:0 / max:10), maximum retraction moves the cylinder 1mm
int Speed;                   // speed of motor(min: -255 / max :255), minus means direction
byte Backlash;               // backlash
float Pitch = 0.4;           // pitch of M4 bolt.
int gearRatio = 50;          // gear ratio of geared motor.
byte X, Y;                   // coordinate of OLED screen

unsigned long current_Millis, previous_Millis = 0;  // Time variable to check the time the switch is pressed
unsigned long pressed_Time = 500;  // Time-based variable. If it is longer than this, it is considered as a LONG press.


long initialPosition;            // PnP 모드에서 엔코더의 현재 위치 저장 변수


U8GLIB_SSD1306_128X32 u8g(U8G_I2C_OPT_FAST);  // I2C / TWI
Encoder myEncoder(encoderA, encoderB);

void setup() {
  pinMode(up_Pin, INPUT_PULLUP);
  pinMode(sel_Pin, INPUT_PULLUP);
  pinMode(down_Pin, INPUT_PULLUP);
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);

  pinMode(AIN_Pin_1, OUTPUT);
  pinMode(AIN_Pin_2, OUTPUT);

  u8g.setFont(u8g_font_helvB10);

  u8g.setFontRefHeightExtendedText();
  u8g.setDefaultForegroundColor();
  u8g.setFontPosTop();

  Serial.begin(9600);
  Serial.println("OK");

  if (EEPROM.read(0) == 255) {  // if there are no saved settings
    Extrusion = 10;             // use default value
    Retraction = 1;
    Speed = 255;
    // 초기화 완료 표시를 위해 임의의 값을 EEPROM의 첫 번째 위치에 기록
    EEPROM.write(0, 0);
    EEPROM.write(1, Extrusion);
    EEPROM.write(2, Retraction);
    EEPROM.write(3, Speed);
  } else {
    Extrusion = EEPROM.read(1);
    Retraction = EEPROM.read(2);
    Speed = EEPROM.read(3);
  }

  draw();
}

void loop() {
  newPosition = myEncoder.read();
  current_Millis = millis();

  // Stage에 따른 동작 처리
  switch (Stage) {
    case 0:  // manual stage
      // 스위치 상태 읽기
      if(digitalRead(up_Pin) == LOW) {
        digitalWrite(AIN_Pin_1, Speed);
        digitalWrite(AIN_Pin_2, LOW);
      }
      if(digitalRead(down_Pin) == LOW) {
        digitalWrite(AIN_Pin_1, LOW);
        digitalWrite(AIN_Pin_2, Speed);
      }
      if((digitalRead(up_Pin) == HIGH) && (digitalRead(down_Pin) == HIGH)){
        stopMotor();
      }
      
      // sel 스위치 처리 함수 호출
      handleSelSwitch();
      break;

    case 1:  // auto stage
      // up 스위치를 누르면 - extrusion 거리 만큼 전진한 뒤 retraction 거리만큼 후진하고 멈춘다.
      if (!up_Flag && digitalRead(up_Pin) == LOW) {
        up_Flag = true;
      } else if (up_Flag && digitalRead(up_Pin) == HIGH) {
        up_Flag = false;
        moveMotor(Extrusion);
        moveMotor(-Retraction);
        stopMotor();
      }

      // down 스위치를 눌렀다 떼면 - extrusion 거리 만큼 후진하고 멈춘다.
      if (!down_Flag && digitalRead(down_Pin) == LOW) {
        down_Flag = true;
      } else if (down_Flag && digitalRead(down_Pin) == HIGH) {
        down_Flag = false;
        moveMotor(-Extrusion);
        stopMotor();
      }

      // sel 스위치를 처리 함수 호출
      handleSelSwitch();
      break;

    case 2:  // Pick and Place stage
      // up 스위치를 누르면 - 스위치를 누른 동안 계속 후진하다가 스위치를 떼면 다시 현재 위치까지 돌아오고 멈춘다.
      if (!up_Flag && digitalRead(up_Pin) == LOW) {
        up_Flag = true;
        initialPosition = myEncoder.read();  // 현재 위치 저장
      } else if (up_Flag && digitalRead(up_Pin) == LOW) {
        // 스위치가 눌린 동안 계속 후진
        digitalWrite(AIN_Pin_1, LOW);
        analogWrite(AIN_Pin_2, Speed);
      } else if (up_Flag && digitalRead(up_Pin) == HIGH) {
        // 스위치가 떼지면 최대 속도로 기존의 위치로 돌아옴
        up_Flag = false;
        long target = initialPosition;
        while (newPosition != target) {
          newPosition = myEncoder.read();
          if (newPosition < target) {
            analogWrite(AIN_Pin_1, 255);  // 최대 속도로 전진
            digitalWrite(AIN_Pin_2, LOW);
          } else {
            digitalWrite(AIN_Pin_1, LOW);
            analogWrite(AIN_Pin_2, 255);  // 최대 속도로 후진
          }
        }
        stopMotor();
      }

      // sel 스위치를 처리 함수 호출
      handleSelSwitch();
      break;

    case 4:  // setup stage
      // up 스위치 상태 읽기
      if (!up_Flag && digitalRead(up_Pin) == LOW) {
        up_Flag = true;
        if (setup_Stage == 0 && Extrusion < 100) {
          Extrusion++;
          draw(); // 화면 업데이트
        } else if (setup_Stage == 1 && Retraction < 10) {
          Retraction++;
          draw(); // 화면 업데이트
        } else if (setup_Stage == 2) {
          if (Speed < 255) {
            Speed += 26;  // 0~255 범위에서 10단계로 변하기 위해
            if (Speed > 255) Speed = 255;
            draw(); // 화면 업데이트
          }
        }
      } else if (up_Flag && digitalRead(up_Pin) == HIGH) {
        up_Flag = false;
      }

      // down 스위치 상태 읽기
      if (!down_Flag && digitalRead(down_Pin) == LOW) {
        down_Flag = true;
        if (setup_Stage == 0 && Extrusion > 0) {
          Extrusion--;
          draw(); // 화면 업데이트
        } else if (setup_Stage == 1 && Retraction > 0) {
          Retraction--;
          draw(); // 화면 업데이트
        } else if (setup_Stage == 2) {
          if (Speed > 0) {
            Speed -= 26;  // 0~255 범위에서 10단계로 변하기 위해
            if (Speed < 0) Speed = 0;
            draw(); // 화면 업데이트
          }
        }
      } else if (down_Flag && digitalRead(down_Pin) == HIGH) {
        down_Flag = false;
      }

      // sel 스위치를 처리 함수 호출
      handleSelSwitch();
      break;

  }

  // 위치 변경 시 OLED 화면 업데이트
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    draw();
  }
}

void stopMotor() {
    digitalWrite(AIN_Pin_1, HIGH);
  digitalWrite(AIN_Pin_2, HIGH);
}

// 모터를 지정된 거리만큼 이동시키는 함수
void moveMotor(int distance) {
  // 이동할 거리를 엔코더 신호 수로 변환
  int counts = (distance * 12 * gearRatio) / Pitch;
  long target = newPosition + counts;

  while (newPosition != target) {
    newPosition = myEncoder.read();
    if (counts > 0) {
      digitalWrite(AIN_Pin_1, Speed);
      digitalWrite(AIN_Pin_2, LOW);
    } else {
      digitalWrite(AIN_Pin_1, LOW);
      digitalWrite(AIN_Pin_2, Speed);
    }
  }
}

// sel 스위치 입력 처리 함수
void handleSelSwitch() {
  if (!select_Flag && digitalRead(sel_Pin) == LOW) {
    select_Flag = true;
    previous_Millis = current_Millis;  // 버튼을 누른 시간 저장
  } else if (select_Flag && digitalRead(sel_Pin) == HIGH) {
    unsigned long pressDuration = current_Millis - previous_Millis;

    if (pressDuration < pressed_Time) {
      // Short press: Stage 변경 또는 setup_Stage 순환
      if (Stage == 4) {
        setup_Stage = (setup_Stage + 1) % 3; // setup stage 순환
      } else {
        Stage = (Stage + 1) % 3; // 0, 1, 2 순환
      }
    } else {
      // Long press: Setup stage로 전환 또는 이전 Stage로 복귀
      if (Stage == 4) {
        // Setup stage에서 벗어날 때 EEPROM에 값 저장
        EEPROM.update(1, Extrusion);
        EEPROM.update(2, Retraction);
        EEPROM.update(3, Speed);
        
        Stage = previous_Stage;  // 이전 Stage로 복귀
      } else {
        previous_Stage = Stage;  // 현재 Stage 저장
        Stage = 4;               // Setup stage로 전환
      }
    }
    draw();
    select_Flag = false;
  }
}


void draw() {
  u8g.firstPage();
  do {
    int strWidth;
    int strHeight = u8g.getFontAscent() - u8g.getFontDescent();
    int x, y;

    switch (Stage) {
      case 0:  // manual stage, motor runs when up/down switch pressed.
        strWidth = u8g.getStrWidth("MANUAL");
        x = (128 - strWidth) / 2;  // 화면 가로 중앙
        y = (32 - strHeight) / 2;  // 화면 세로 중앙
        u8g.drawStr(x, y, "MANUAL");
        break;

      case 1:  // auto stage, The motor runs for a set amount.
        strWidth = u8g.getStrWidth("AUTO");
        x = (128 - strWidth) / 2;  // 화면 가로 중앙
        y = (32 - strHeight) / 2;  // 화면 세로 중앙
        u8g.drawStr(x, y, "AUTO");
        break;

      case 2:  // Pick and Place stage, motor runs forward when UP switch pressed and backward when released.
        strWidth = u8g.getStrWidth("PnP");
        x = (128 - strWidth) / 2;  // 화면 가로 중앙
        y = (32 - strHeight) / 2;  // 화면 세로 중앙
        u8g.drawStr(x, y, "PnP");
        break;

      case 4:  // Setup stage, you can adjust settings here.
        switch (setup_Stage) {  // 0 = extrusion, 1 = retraction, 2 = speed
          case 0:
            {
              String extrusionText = "Extrusion: " + String(Extrusion);
              strWidth = u8g.getStrWidth(extrusionText.c_str());
              x = (128 - strWidth) / 2;  // 화면 가로 중앙
              y = 0;  // 화면 세로 제일 위
              u8g.drawStr(x, y, extrusionText.c_str());
            }
            u8g.drawFrame(0, 17, 128, 14);
            u8g.drawBox(0, 17, Extrusion, 14);
            break;
          case 1:
            {
              String retractionText = "Retraction: " + String(Retraction);
              strWidth = u8g.getStrWidth(retractionText.c_str());
              x = (128 - strWidth) / 2;  // 화면 가로 중앙
              y = 0;  // 화면 세로 제일 위
              u8g.drawStr(x, y, retractionText.c_str());
            }
            u8g.drawFrame(0, 17, 128, 14);
            u8g.drawBox(0, 17, Retraction, 14);
            break;
          case 2:
            {
              int mappedSpeed = map(Speed, 0, 255, 1, 10);  // Speed 값을 1~10으로 매핑
              String speedText = "Speed: " + String(mappedSpeed);
              strWidth = u8g.getStrWidth(speedText.c_str());
              x = (128 - strWidth) / 2;  // 화면 가로 중앙
              y = 0;  // 화면 세로 제일 위
              u8g.drawStr(x, y, speedText.c_str());
            }
            u8g.drawFrame(0, 17, 128, 14);
            u8g.drawBox(0, 17, map(Speed, 0, 255, 0, 128), 14);  // Speed 값을 0~128으로 매핑하여 막대그래프로 표시
            break;
        }
        break;
    }
  } while (u8g.nextPage());
}
