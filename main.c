#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>
#include "BluetoothSerial.h"  //Bluetooth library import
#include "DHT.h"              //DHT22 library

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
//

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define BUTTON_1_PIN 35   // Button 1 Pin 35
#define BUTTON_2_PIN 32   // Button 2 Pin 35
#define HALL_EFFECT 33    //Hall effects
#define DHTPIN 26         //define DHT22 pin

#define DHTTYPE DHT22  //Define DHT edition in this case 22

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);  //create OLED instance

Adafruit_BMP085 bme;  //create BMP180 instance

BluetoothSerial SerialBT;  //create bluetooth instance
//Max30102 start
MAX30105 particleSensor;  //create MAX10302 instance
String RcvdCmd;           //Store recived bluetooth Command

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
int menu_pos = 0;

//bluetooth
int Blt = 1;

//Time last hall effect sense occured
double lsthalls = 0;
float speed = 0;


//Hardware Intrupt Button--Start
struct Button {
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  bool pressed;
};

Button button1 = { BUTTON_1_PIN, 0, false };
Button button2 = { BUTTON_2_PIN, 0, false };
Button hall_effect = { HALL_EFFECT, 0, false };





void IRAM_ATTR isr() {
  button1.numberKeyPresses++;
  button1.pressed = true;
  //Serial.println("Hall Effect detected");
}

void IRAM_ATTR isr2() {
  button2.numberKeyPresses++;
  button2.pressed = true;
  //Serial.println("Hall Effect detected");
}

void IRAM_ATTR isr3() {
  hall_effect.numberKeyPresses++;
  hall_effect.pressed = true;
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

//Hall Effect




//Run at power Up
void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32");  //Bluetooth device name
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  //pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  //pinMode(HALL_EFFECT, INPUT_PULLUP);  //Hall effects

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    oled = 0;
  }

  //Button 1&2
  pinMode(button1.PIN, INPUT_PULLUP);
  attachInterrupt(button1.PIN, isr, FALLING);

  pinMode(button2.PIN, INPUT_PULLUP);
  attachInterrupt(button2.PIN, isr2, FALLING);

  //Hall Effect
  pinMode(hall_effect.PIN, INPUT_PULLUP);
  attachInterrupt(hall_effect.PIN, isr3, FALLING);




  //BMP180 startup
  bool status = bme.begin(0x77);
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    bmp = 0;
  }

  dht.begin();  //DHT22 begin sendig data

  //OLED startup config
  delay(2000);
  display.clearDisplay();
  display.setTextColor(WHITE);

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

//Run as loop
//Task1code:Button & speedmeter(Underdevelopment)
void Task1code(void* pvParameters) {
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    long irValue = particleSensor.getIR();

    if (SerialBT.available()) {
      //int data=SerialBT.read();
      //RcvdCmd=String(data);
      Serial.println(SerialBT.read());
    }

    if (button1.pressed) {
      //Serial.printf("Button 1 is pressed");
      menu_pos++;
      button1.pressed = false;
    }

    if (button2.pressed) {
      Serial.printf("Button 2 is pressed");
      menu_pos--;
      button2.pressed = false;
    }

    if (hall_effect.pressed) {
      Serial.printf("Hall effect is detected");
      lsthalls = millis() - lsthalls;
      speed = (1000.0 / lsthalls * hall_effect.numberKeyPresses);
      //Serial.print(speed);
      //Serial.print(speed);
      lsthalls = millis();
      hall_effect.pressed = false;
    }

    if (checkForBeat(irValue) == true) {

      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      if (irValue < 50000) {
        beatsPerMinute = 0;
      } else {
        beatsPerMinute = 60 / (delta / 1000.0);
      }


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
    //RcvdCmd = String(SerialBT.read());
  }
}

//Run as loop
//Task2code:Display & bluetooth
void Task2code(void* pvParameters) {
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for (;;) {
    //DHT22 Reading
    float hum = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float temp = dht.readTemperature();


    display.clearDisplay();
    // display temperature on OLED
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
        display.print(RcvdCmd);
        break;
      case 1:
        display.print("HeartRate: ");
        display.setTextSize(2);
        display.setCursor(0, 10);
        display.print(String(bme.readAltitude()));
        display.print(" ");
        display.setTextSize(1);
        display.cp437(true);
        display.write(167);
        display.setTextSize(2);
        display.print("C");
        break;
      case 2:
        display.print("Speed: ");
        display.setTextSize(2);
        display.setCursor(0, 10);
        display.print(String(speed));
        display.print(" ");
        display.setTextSize(1);
        display.cp437(true);
        display.write(167);
        display.setTextSize(2);
        display.print("C");
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
    display.display();

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis;
      temperatureString = String(bme.readTemperature());
      //SerialBT.println(temperatureString);
      //Altitude
      altString = String(bme.readAltitude());
    }

    if (Blt == 0) {
      if (Serial.available()) {
        SerialBT.println(temperatureString + "|" + altString + "|" + beatsPerMinute + "|" + hum + "|" + temp + "|" + speed);
      }
    }

    // Send temperature readings via bluetooth communication
    delay(1000);
  }
}

void loop() {
}