#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

#define pinT1 23 // analog pin
#define pinT2 22 // analog pin

//Konfigurasi Pin Analog Sensor Arus ACS712
// #define pinACS 27

// Konfigurasi Pin Digital Output ESP32
#define pinSpeed 25
#define pinDirection 26

//Konstanta
#define Vcc 1023 
#define Kt 1.25 
#define Kp 17 //21 
#define MAXPWM 255 
#define MINPWM 0 
#define outputReading 5

//Global Scope
unsigned long waktu_lama = 0,waktu_baru = 0, delta = 0;
double Steering_Torque = 0; // Torsi Roda Kemudi
double Motor_Torque = 0; // Torsi Motor BLDC
float Motor_Speed = 0; // Kecepatan Motor
double Total_Torque = 0; // Torsi Total

//filter output
double output_reading[outputReading], output_total = 0;
double output_average = 0;
int output_readIndex = 0;

//PID Variable
double set_point = 0;
double error = 0;
double I_set = 0;
double pre_error = 0;
double pwm_set = 0;
bool dir = 0;

//Speed Variable
volatile int rev=0;
float rpm;
float realRPM = 0;
float oldtime=0;
float newtime;

//Speed Variable
int x = 1;
unsigned long d1, d2;
const int numReadings = 15;
int readings [numReadings];
int readIndex = 0;
float total = 0;

//variable print
float nilaiTegangan, nilaiArus, motorTorque;

//konfigurasi lcd
LiquidCrystal_I2C lcd(0x27, 16, 2);

//konfigurasi rtos
TaskHandle_t mainTask;
TaskHandle_t displayTask;

void IRAM_ATTR isr(){
  rev++;
}

void InputOutputInit(){
  // Input Pin
  pinMode(pinT1, INPUT);
  pinMode(pinT2, INPUT);
  // pinMode(pinACS, INPUT);
  // Output Pin
  pinMode(pinSpeed, OUTPUT);
  pinMode(pinDirection, OUTPUT);
}

void moving_averageInit(){
  for(int i = 0; i < outputReading; i++){
  output_reading[i] = 0;
  }
}

float Calc_Motor_Torque(){
  int bacaArus = analogRead(27);
  nilaiTegangan = (bacaArus/4095.0) * 3300;
  nilaiArus = ((nilaiTegangan - 2380) / 100);
  motorTorque = nilaiArus * Kt;
  return motorTorque;
}

float moving_average(float data){
  float average;
  total = total - readings[readIndex];
  readings[readIndex] = data;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  if (readIndex >= numReadings) {
    readIndex = 0;
  }
  average = total / numReadings;
  return average;
}

float Calc_Motor_Speed(){
  float wings, RPMnew;
  d2 = millis();
	if(d2-d1 >= 300){
		x=1-x;
		d1=millis();
	}
	switch(x){
		case 0:
		{
			break;
		}
		case 1:
			detachInterrupt(34); 
			newtime=millis()-oldtime;  
		  wings= 20; 
			RPMnew = rev/wings; 
			rpm = (RPMnew/newtime*60000);
			oldtime=millis(); 
			rev=0;
			attachInterrupt(34,isr,RISING);
			break;
	}
	if(rpm > 0 && rpm < 180){
		realRPM = rpm;
	}
	else{;;}
	float rpmFilter = moving_average(realRPM);
  return rpmFilter;
}

float Calc_Steering_Torque(){
  float persenT1, persenT2, reldegree, reldegree1, reldegree2, torsi;
  int T11 = analogRead(33);
  int T22 = analogRead(32);
  persenT1 = 100*(T11/4095.0);
  persenT2 = 100*(T22/4095.0);
  reldegree1 = (0.2 * persenT1) - 9.614;
  reldegree2 = (-0.2 * persenT2) + 10.1461;
  reldegree = (reldegree1 + reldegree2) * 0.5;
  torsi = reldegree * 1.547;
  return torsi; 
}

// untuk mendapatkan output pwm rata-rata
double filter_output(double value){
  output_total -= output_reading[output_readIndex];
  output_reading[output_readIndex] = value;
  output_total += output_reading[output_readIndex];
  output_readIndex++;
  if(output_readIndex >= outputReading){
    output_readIndex = 0;
  }
  return (output_total/outputReading);
}

// koefisien daya bantuan yang merupakan fungsi kecepatan.
double Kv(double v){
  return 3.475 - 0.0606*v + 0.0003*v*v;
  // return 4.0 - 0.0606*v + 0.0003*v*v;
}

// untuk mendapatkan nilai setpoint dari stir
double Calc_Set_Point(double Td, double v){
  if(Td > 7) Td = 7;
  if(Td < 2) return ((Kv(v)/4)*Td*Td);
  else return (Kv(v)*(Td-1));
}

// fungsi jika kondisi stir diputer searah jarum jam
void clockwise(int value, bool enable){
  digitalWrite(pinDirection, 0);
  analogWrite (pinSpeed, value); 
}
// fungsi jika kondisi stir diputar berlawanan jarum jam
void counter_clockwise(int value, bool enable){
  digitalWrite(pinDirection, 1);
  analogWrite (pinSpeed, value); 
}
// motor berhenti
void stop_motor(){
  digitalWrite(pinDirection, 0);
  analogWrite (pinSpeed, 0); 
}

void debug(){
  float mtrspeed = Calc_Motor_Speed();
  float mtrtorque = Calc_Motor_Torque();
  float strtorque = Calc_Steering_Torque();

  Serial.print("Speed = ");
  Serial.println(mtrspeed);
  Serial.print("Motor Torque = ");
  Serial.println(mtrtorque);
  Serial.print("Steering Torque = ");
  Serial.println(strtorque);
}

void mainTaskcode(void *pvParameters){
  for(;;){
    Motor_Speed = Calc_Motor_Speed();
    Motor_Torque = Calc_Motor_Torque();
    Steering_Torque = Calc_Steering_Torque();
    
    if(Motor_Speed <= 60){
      Motor_Speed = 0;
    }
    else{
      Motor_Speed = Motor_Speed * 1;
    }
    set_point = Calc_Set_Point(abs(Steering_Torque), Motor_Speed);

    if (Steering_Torque >= 0) dir = 0;
    else if (Steering_Torque < 0) dir = 1;

    Total_Torque = abs(Steering_Torque) + Motor_Torque;
    error = set_point - Motor_Torque;
    pwm_set = Kp*error;
    output_average = filter_output(pwm_set);

    if(output_average < MINPWM){ 
      output_average = MINPWM;
    }
    else if(output_average > MAXPWM) {
      output_average = MAXPWM;
    }

    if(dir == 0)clockwise(output_average,1);
    else if(dir == 1){
    counter_clockwise(output_average,1);
    }
  }
}

void displayTaskcode(void *pvParameters){
  for(;;){
    lcd.setCursor(0, 0);
    lcd.print("S: ");
    lcd.setCursor(2, 0);
    lcd.print(Motor_Speed);
    lcd.setCursor(10, 0);
    lcd.print("MT: ");
    lcd.setCursor(13, 0);
    lcd.print(Motor_Torque);
    lcd.setCursor(0, 1);
    lcd.print("ST: ");
    lcd.setCursor(3, 1);
    lcd.print(Steering_Torque);
    lcd.setCursor(9, 1);
    lcd.print("PWM: ");
    lcd.setCursor(13, 1);
    lcd.print(output_average);
    Serial.print(nilaiArus);
    Serial.print(";");
    Serial.print(motorTorque);
    Serial.print(";");
    Serial.print(Steering_Torque);
    Serial.print(";");
    Serial.println(output_average);
    // vTaskDelay(100);
    delay(100);
    lcd.clear();
  }
}

void setup() {
  Serial.begin(9600, SERIAL_8N1);
  InputOutputInit();
  lcd.init();
  lcd.backlight();
  attachInterrupt(34, isr, RISING);
  moving_averageInit();

  xTaskCreatePinnedToCore(
    mainTaskcode,
    "mainTask",
    10000,
    NULL,
    1,
    &mainTask,
    1
  );

  xTaskCreatePinnedToCore(
    displayTaskcode,
    "displayTask",
    10000,
    NULL,
    1,
    &displayTask,
    0
  );
}

void loop() {
  
}
