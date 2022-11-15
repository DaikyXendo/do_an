#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define pulX 5  //define Pulse pin
#define dirX 4   //define Diression pin
#define enaX 3    //define Enable Pin
#define pulY 6
#define dirY 7
#define enaY 8

#define ssBottomPin 12
#define ssTopPin 11
#define ssLeftPin 10
#define ssRightPin 9

#define motorX 1
#define motorY 2

float speed = 30;
float i = 0, j = 0;

const byte diachi[6] = "12345";       // địa chỉ phát
RF24 radio(2,13);

void stepperSetup();
void stepperRun(uint8_t motorNumber, float motorSpeed);
void goUp();
void goDown();
void goLeft();
void goRight();
void goUpRight();
void goUpLeft();
void goDownRight();
void goDownLeft();
void stopp();

void setup() {
  Serial.begin(9600);
  stepperSetup();
  pinMode(ssRightPin, OUTPUT);
  pinMode(ssLeftPin, OUTPUT);

  pinMode(pulX, OUTPUT);
  pinMode(pulY, OUTPUT);

  pinMode(enaX, OUTPUT);
  pinMode(enaY, OUTPUT);

  pinMode(dirX, OUTPUT);
  pinMode(dirY, OUTPUT);

  // Cong tac
  pinMode(ssTopPin, INPUT);
  pinMode(ssBottomPin, INPUT);
  pinMode(ssLeftPin, INPUT);
  pinMode(ssRightPin, INPUT);
  if (!radio.begin()) 
  {
    Serial.println("Module không khởi động được...!!");
    while (1) {}
  }
  radio.openReadingPipe(1, diachi);
  //Lệnh openReadingPipe có số đường truyền từ 0-5
  //Nhưng đường 0 đã được dùng cho ghi (mặc định)
  //Vì vậy chỉ dùng 1-5, nếu dùng 0 sẽ bị chồng lấn
  //Mở 1 kênh có địa chỉ 12345 trên đường truyền 1
  //kênh này chỉ đọc data trên địa chỉ 12345   
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(80);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();
  
  if (!radio.available())
  {
    Serial.println("Chưa kết nối được với TX...!!");
    Serial.println("CHỜ KẾT NỐI.......");
  } 
}

void loop() {
  stepperRun(motorY, speed);

  int top = digitalRead(ssTopPin);
  int bottom = digitalRead(ssBottomPin);
  int left = digitalRead(ssLeftPin);
  int right = digitalRead(ssRightPin);

  delay(200);

  int top1 = digitalRead(ssTopPin);
  int bottom1 = digitalRead(ssBottomPin);
  int left1 = digitalRead(ssLeftPin);
  int right1 = digitalRead(ssRightPin);

  if (bottom == 1 && bottom1 == 1) {
    stopp();
    goUp();
  };

  if (left == 1 && left1 == 1) {
      digitalWrite(enaX, HIGH); 
  };

  if (right == 1 && right1 == 1) {
      digitalWrite(enaX, HIGH); 
  };

  if (radio.available()) 
  {
    char nhan_str[5] = "";
    radio.read(&nhan_str, sizeof(nhan_str));
    int nhan_int = atoi(nhan_str);
    Serial.print("**");
    Serial.println(nhan_int);
    switch(nhan_int)
    {
      case 1: 
      {
          goUp();
          break;
      }
      case 2:
      {
          stopp();
          break;
      }
      case 3:
      {
          goDown();
          break;
          Serial.println("#######");
      }
      case 4: 
      {
          break;
      }
      default: break;
    }
  }
}

void goUp() {
  speed = 30;
  delay(50);
  stepperRun(motorY, speed);
  Serial.println("go up");
}

void goDown() {
  speed = -30;
  delay(50);
  stepperRun(motorY, speed);
  Serial.println("go down");
}

void goLeft() {
  speed = -30;
  delay(50);
  stepperRun(motorX, speed);
  Serial.println("go left");
}

void goRight() {
  speed = 30;
  delay(50);
  stepperRun(motorX, speed);
  Serial.println("go right");
}

void goUpRight() {
  speed = 30;
  delay(50);
  stepperRun(motorX, speed);
  stepperRun(motorY, speed);
  Serial.println("go up right");
}

void goUpLeft() {
  speed = 30;
  delay(50);
  stepperRun(motorX, speed);
  speed = -30;
  stepperRun(motorY, speed);
  Serial.println("go up left");
}

void goDownRight() {
  speed = -30;
  delay(50);
  stepperRun(motorX, speed);
  speed = 30;
  stepperRun(motorY, speed);
  Serial.println("go down right");
}

void goDownLeft() {
  speed = -30;
  delay(50);
  stepperRun(motorX, speed);
  stepperRun(motorY, speed);
  Serial.println("go down left");
}

void goCircle() {
  int x = 0;

  while(x <= 360) {
    speed = (int)(30 * sin(x * 3.1416 / 180));
    stepperRun(motorX, speed);
    stepperRun(motorY, speed);
    delay(50);
    x += 2;
  }
}

void stopp() {
  speed = 0;
  Serial.println("stop");
  stepperRun(motorY, speed);
  delay(1000);
}

void stepperSetup() {
  pinMode(pulX, OUTPUT);
  pinMode(pulY, OUTPUT);

  pinMode(enaX, OUTPUT);
  pinMode(enaY, OUTPUT);

  pinMode(dirX, OUTPUT);
  pinMode(dirY, OUTPUT);

  digitalWrite(enaX, HIGH);  // disable motor
  digitalWrite(enaY, HIGH);  // disable motor

  //  digitalWrite(dirX, LOW); // set initial motor direction
  //  digitalWrite(dirY, LOW); // set initial motor direction

  cli();  //stop interrupts

  TCCR3A = 0;
  TCCR3B = 0;
  //control override for pin OC3A (IDE pin 5) Toggle OC3A on Compare Match, fast PWM with OCR3A as TOP
  TCCR3A = _BV(COM3A0) | _BV(WGM31) | _BV(WGM30);  
  TCCR3B = _BV(CS30) | _BV(WGM33) | _BV(WGM32);    //internal clock with CLK/1 prescaling

  TCCR4A = 0;
  TCCR4B = 0;
  //control override for pin OC4A (IDE pin 6) Toggle OC4A on Compare Match, fast PWM with OCR4A as TOP
  TCCR4A = _BV(COM4A0) | _BV(WGM41) | _BV(WGM40);  
  TCCR4B = _BV(CS40) | _BV(WGM43) | _BV(WGM42);    //internal clock with CLK/1 prescaling

  sei();  // Start Interrupt

  OCR3A = 0;  //IDE pin 5 step_b_pin
  OCR4A = 0;  //IDE pin 6 step_a_pin
}

void stepperRun(uint8_t motorNumber, float motorSpeed) {
  bool dir;  //motor direction
  uint16_t PWM_count;

  //determine needed motor direction and make speed pos.
  if (motorSpeed < 0) {
    dir = 0;
    motorSpeed = abs(motorSpeed);

  } else {
    dir = 1;
  }

  if (motorSpeed > 0) {
    PWM_count = 5000 / motorSpeed;  //calculate needed compare value for speed
  } else PWM_count = 0;              //set compare value to 0 to keep stepper output from toggling when zero velocity needed

  if (motorNumber == 1)  //motor a-------------------------------------------
  {
    digitalWrite(dirX, dir);
    OCR3A = PWM_count;  //set motor a step speed compare register
  }

  if (motorNumber == 2)  //motor b----------------------------------------------
  {
    digitalWrite(dirY, dir);
    OCR4A = PWM_count;  //set motor b step speed compare register
  }
}
