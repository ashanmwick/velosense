#include <Wire.h>              //I2C communication
#include <Adafruit_GFX.h>      //OLED screen
#include <Adafruit_SSD1306.h>  ////OLED screen
#include <Adafruit_Sensor.h>   //BMP180
#include <Adafruit_BMP085.h>   //BMP180
#include "BluetoothSerial.h"   //Bluetooth library import
#include "DHT.h"               //DHT22 library

//MAX10302
#include "MAX30105.h"
#include "heartRate.h"



//----Current implemented-----//
//Hall effect
//OLED
//Bluetooth

//-----To Do
//Heart Rate
//Temperature
//Preasure altitude
//Humidity

//Logo Btmap
const unsigned char logoMischianti[1024] PROGMEM = {
  // 'logoBN128x64, 128x64px
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xc6, 0x00, 0x01, 0xff, 0xfe, 0x0e, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x86, 0x00, 0x11, 0xff, 0xf8, 0x02, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x04, 0x3f, 0x0c, 0x00, 0x01, 0xff, 0xe0, 0x02, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x04, 0x3e, 0x1c, 0x7f, 0xe3, 0xff, 0xc3, 0xc1, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x04, 0x1c, 0x3c, 0x7f, 0xe3, 0xff, 0x87, 0xe1, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x04, 0x1c, 0x78, 0x7f, 0xc3, 0xff, 0x0f, 0xe1, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0c, 0x18, 0x78, 0x00, 0xc7, 0xff, 0x0f, 0xe1, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0c, 0x10, 0xf8, 0x01, 0xc7, 0xfe, 0x1f, 0xe1, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0e, 0x01, 0xf0, 0x01, 0x87, 0xfe, 0x1f, 0xc3, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0e, 0x03, 0xf1, 0xff, 0x8f, 0xfe, 0x1f, 0xc3, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0e, 0x07, 0xf1, 0xff, 0x8f, 0xfe, 0x1f, 0x87, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0e, 0x07, 0xe3, 0xff, 0x1f, 0xfe, 0x0f, 0x0f, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0e, 0x0f, 0xe0, 0x00, 0x00, 0x07, 0x00, 0x1f, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0e, 0x1f, 0xe0, 0x01, 0x00, 0x0f, 0x00, 0x7f, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x0e, 0x3f, 0xc0, 0x00, 0x00, 0x0f, 0xc1, 0xff, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xf8, 0x00, 0x08, 0xe2, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xf0, 0x00, 0x18, 0xe4, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xe7, 0xe3, 0xf0, 0x61, 0xf8, 0xfe, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xe0, 0x20, 0x70, 0x48, 0x08, 0x1e, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xf0, 0x00, 0x70, 0x0c, 0x00, 0x1e, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0x0f, 0xe6, 0x0f, 0xc3, 0xfe, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0x0f, 0xe6, 0x1f, 0xc3, 0xfc, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xc0, 0x00, 0x67, 0x10, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0x80, 0x80, 0x4f, 0x00, 0x20, 0x18, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

//Macros
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define BUTTON_1_PIN 35   // Button 1 Pin 35
#define BUTTON_2_PIN 32   // Button 2 Pin 35
#define HALL_EFFECT 33    //Hall effects
#define DHTPIN 26         //define DHT22 pin

#define DHTTYPE DHT22  //Define DHT edition in this case 22

//Timer starts here
double start_time;
double diff_ms_time = 0;
int minute_time = 0;
int seco_time = 0;
int hrs_time = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);  //create OLED instance

Adafruit_BMP085 bme;  //create BMP180 instance

BluetoothSerial SerialBT;  //create bluetooth instance
//Max30102 start
MAX30105 particleSensor;  //create MAX10302 instance

String RcvdCmd;  //Store recived bluetooth Command

DHT dht(DHTPIN, DHTTYPE);  //create dht 22 instance

const byte RATE_SIZE = 4;  //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];     //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0;  //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
//Max30102 end

//Sesnor check variables
int maxs = 1;
int oled = 1;
int bmp = 1;

//menu position
int menu_pos = 2;

//bluetooth
int Blt = 0;

//Time last hall effect sense occured
double lsthalls = 0;
double time_gap = 0;
float speed = 0;
float speed_val = 0;


//Hardware Intrupt Button--Start
struct Button {
  const uint8_t PIN;//uint8_t should check
  uint32_t numberKeyPresses;//Not used
  bool pressed;//Not used
};

Button button1 = { BUTTON_1_PIN, 0, false };
Button button2 = { BUTTON_2_PIN, 0, false };
Button hall_effect = { HALL_EFFECT, 0, false };




//Hardware Intruption
void IRAM_ATTR isr() {//Button1
  //button1.numberKeyPresses++;
  //button1.pressed = true;
  menu_pos++;
  //button1.pressed = false;
  //Serial.println("Hall Effect detected");
}

void IRAM_ATTR isr2() {//Button2
  //button2.numberKeyPresses++;
  //button2.pressed = true;
  //Serial.println("Hall Effect detected");
  menu_pos--;
  //button1.pressed = false;
}

void IRAM_ATTR isr3() {//Hall Effect Sensor
  //hall_effect.numberKeyPresses++;
  time_gap = millis() - lsthalls;
  lsthalls = millis();
  speed = (1000.0 / time_gap);
  hall_effect.pressed = true;
  speed_val = (2 * 3.14 * 0.15 * speed) * 3.6;//r=0.15 
  //Serial.println("Hall Effect detected");
}



// Using for Thread
TaskHandle_t Task1;
TaskHandle_t Task2;

//Using for temperature & altitude reading
String altString = "";
String temperatureString = "";
unsigned long previousMillis = 0;  // Stores last time temperature was published
const long interval = 10000;       // interval at which to publish sensor readings


//Run at power Up
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32");  //Bluetooth device name
  //pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  //pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  //pinMode(HALL_EFFECT, INPUT_PULLUP);  //Hall effects

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {//0x3C is oled address
    Serial.println(F("SSD1306 allocation failed"));
  }
  //OLED startup config
  //delay(2000);
  display.clearDisplay();


  //Button 1&2
  pinMode(button1.PIN, INPUT_PULLUP);//Get pin inputs
  attachInterrupt(button1.PIN, isr, FALLING);//Faling refers to Voltage Falling

  pinMode(button2.PIN, INPUT_PULLUP);//Get pin inputs
  attachInterrupt(button2.PIN, isr2, FALLING);//Faling refers to Voltage Falling

  //Hall Effect
  pinMode(hall_effect.PIN, INPUT_PULLUP);//Get pin inputs
  attachInterrupt(hall_effect.PIN, isr3, FALLING);//Faling refers to Voltage Falling


  //BMP180 startup
  bool status = bme.begin(0x77);//I2C address 0x77
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    //bmp = 0;
  }

  dht.begin();  //DHT22 begin sendig data


  //display.clearDisplay();

  // Initialize MAX30102
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))  //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    //maxs = 0;
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup();                     //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A);  //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0);   //Turn off Green LED



  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
    Task1code, /* Task function. */
    "Task1",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task1,    /* Task handle to keep track of created task */
    0);        /* pin task to core 0 */
  delay(500);

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
    Task2code, /* Task function. */
    "Task2",   /* name of task. */
    10000,     /* Stack size of task */
    NULL,      /* parameter of the task */
    1,         /* priority of the task */
    &Task2,    /* Task handle to keep track of created task */
    1);        /* pin task to core 1 */
  delay(500);
}


//Task1code:heartrate
void Task1code(void* pvParameters) {
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {//Run as loop
    long irValue = particleSensor.getIR();
    if (checkForBeat(irValue) == true) {

      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;  //Store this reading in the array
        rateSpot %= RATE_SIZE;                     //Wrap variable

        //Take average of readings
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }


  }
}

//Run as loop
//Task2code:Display & bluetooth
void Task2code(void* pvParameters) {
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  display.drawBitmap(0, 0, logoMischianti, 128, 64, WHITE);
  display.display();
  display.setTextColor(WHITE);
  /*
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("Welcome");
  //display.setCursor(2, 0);
  //display.print("Velo Sense");
  display.setCursor(4, 0);
  display.print("Loading Please Wait");
  display.display();
  */
  delay(5000);

  start_time = millis();  //Timer get start Time

  for (;;) {
    //DHT22 Reading
    float hum = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float temp = bme.readTemperature();

    display.clearDisplay();
    // display temperature on OLED
    //Timer Dsplay
    seco_time = (millis() - start_time) / 1000;
    if (seco_time >= 60) {
      minute_time = seco_time / 60;
      seco_time = seco_time % 60;
      if (minute_time >= 60) {
        hrs_time = minute_time / 60;
        minute_time = minute_time % 60;
      } else {
        hrs_time = 0;
      }
    } else {
      minute_time = 0;
      hrs_time = 0;
    }




    display.setTextSize(1);
    display.setCursor(0, 0);
    if (menu_pos < 0) {
      menu_pos = 3;
    } else if (menu_pos > 3) {
      menu_pos = 0;
    }

    switch (menu_pos) {
      case 0:
        display.print("Temperature: ");
        display.setTextSize(2);
        display.setCursor(0, 10);
        display.print(String(bme.readTemperature()));
        display.print(" ");
        display.setTextSize(1);
        display.cp437(true);
        display.write(167);
        display.setTextSize(2);
        display.print("C");
        display.setCursor(0, 25);
        display.setTextSize(2);
        //display.print(RcvdCmd);
        break;
      case 1:
        display.print("Altitude: ");
        display.setTextSize(2);
        display.setCursor(0, 10);
        display.print(String(bme.readAltitude()));
        display.print("m");
        break;
      case 2:
        display.print("Speed: ");
        display.setTextSize(2);
        display.setCursor(0, 10);
        display.print(String(speed_val));
        display.setTextSize(2);
        display.print("km/h");
        break;
      case 3:
        display.print("Env Temp: ");
        display.setTextSize(2);
        display.setCursor(0, 10);
        display.print(String(beatsPerMinute));
        display.print(" ");
        display.setTextSize(1);
        display.cp437(true);
        display.write(167);
        display.setTextSize(2);
        display.print("C");
        break;
    }


    if (minute_time >= 1) {
      display.setCursor(15, 50);
      display.print(String(hrs_time) + ":" + String(minute_time) + ":" + String(seco_time));

    } else {
      display.setCursor(10, 30);
      display.setTextSize(1);
      display.print(String("Waiting to Start"));
      display.setCursor(10, 42);
      display.print(String("the Session!"));
    }
    //display.println(" ");
    Serial.println(String(display.getCursorX()) + " " + String(display.getCursorY()));
    display.display();

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      temperatureString = String(bme.readTemperature());
      //SerialBT.println(temperatureString);
      //Altitude
      altString = String(bme.readAltitude());
      SerialBT.println(temperatureString + "|" + altString + "|" + beatAvg + "|" + hum + "|" + temp + "|" + speed);
      speed = 0;
      speed_val = 0;
    }

    // Send temperature readings via bluetooth communication
    delay(1000);
  }
}

void loop() {
}