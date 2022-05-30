#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h" //http://librarymanager/All#Adafruit_BME280
#include <OneWire.h>
#include <DallasTemperature.h>
#include  <QubitroMqttClient.h>

// Data wire is plugged into GPIO14 on the Arduino
#define ONE_WIRE_BUS 14

#define DOOR_SENSOR 27

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress insideThermometer;

// Replace the next variables with your SSID/Password combination
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_KEY";

// Replace it with your MQTT Broker IP address or domain
const char* mqtt_server = "broker.qubitro.com";

// Define an ID to indicate the device, If it is the same as other devices which connect the same mqtt server,
// it will lead to the failure to connect to the mqtt server
const char* mqttClientId = "YOUR_QUBITRO_MQTT_CLIENT_ID";

// if need username and password to connect mqtt server, they cannot be NULL.
const char* deviceID = "YOUR_QUBITRO_DEVICE_ID";
const char* deviceToken = "YOUR_QUBITRO_MQTT_PASSWORD";

WiFiClient espClient;
QubitroMqttClient mqttClient(espClient);

Adafruit_BME680 bme;
// Might need adjustments
//#define SEALEVELPRESSURE_HPA (1010.0)

float temperaturebme = 0;
float humidity = 0;
float tempDS = 0;

long lastMsg = 0;

String door_state="";
String previous_state="";
String payload="";


void bme680_init()
{
  Wire.begin();

  if (!bme.begin()) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
//  bme.setPressureOversampling(BME680_OS_4X);
//  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
//  bme.setGasHeater(320, 150); // 320*C for 150 ms
}


void ds18b20_init()
{
   // locate devices on the bus
  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");

  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
   
   // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 9);
 
  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC); 
  Serial.println();
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
void setup()
{
  Serial.begin(115200);

  pinMode(DOOR_SENSOR, INPUT);

  bme680_init();

  ds18b20_init();
  
  // conncet WiFi AP
  setup_wifi();

  // connect mqtt broker
  qubitro_init();
 
  // set LED pin mode
//  pinMode(ledPin, OUTPUT);
}

void qubitro_init() {
  char host[] = "broker.qubitro.com";
  int port = 1883;
  mqttClient.setId(deviceID);
  mqttClient.setDeviceIdToken(deviceID, deviceToken);
  Serial.println("Connecting to Qubitro...");
  if (!mqttClient.connect(host, port))
  {
    Serial.print("Connection failed. Error code: ");
    Serial.println(mqttClient.connectError());
    Serial.println("Visit docs.qubitro.com or create a new issue on github.com/qubitro");
  }
  Serial.println("Connected to Qubitro.");
  mqttClient.subscribe(deviceID);
}

/* conncet WiFi AP
   wait until wifi is connected.
*/
void setup_wifi()
{
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void publishData(String payload)
{
    mqttClient.poll();
    mqttClient.beginMessage(deviceID);
    mqttClient.print(payload);
    mqttClient.endMessage();
}

void loop()
{
  
  //Checking door sensor state
  int door_sensor_state = digitalRead(DOOR_SENSOR);
  if (door_sensor_state == HIGH) {
        door_state="OPEN";
        if (previous_state == "CLOSED") {
          Serial.println("Door open");
          payload = "{\"InsideTemp\":" + String(tempDS) + ",\"OutsideTemp\":" + String(temperaturebme) + ",\"DoorState\":" + "\"" +  door_state + "\"" + "}";
          Serial.println(payload);
          publishData(payload);
        }
        
  } else {
        door_state="CLOSED";
        if (previous_state == "OPEN") {
          Serial.println("Door closed");
          payload = "{\"InsideTemp\":" + String(tempDS) + ",\"OutsideTemp\":" + String(temperaturebme) + ",\"DoorState\":" + "\"" +  door_state + "\"" + "}";
          Serial.println(payload);
          publishData(payload);
        }
  }
  previous_state = door_state;

  long now = millis();
  if (now - lastMsg > 60000) // publish every 60s
  {
    // Tell BME680 to begin measurement.
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }
  Serial.print(F("Reading started at "));
  Serial.print(millis());
  Serial.print(F(" and will finish at "));
  Serial.println(endTime);

  Serial.println(F("You can do other work during BME680 measurement."));
  delay(10); // This represents parallel work.

   // get temperature from DS18B20
    sensors.requestTemperatures(); 
    tempDS = sensors.getTempCByIndex(0);
     if(tempDS == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
  }
   //Try reading BME sensor
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }
  Serial.print(F("Reading completed at "));
  Serial.println(millis());

  Serial.println();

    // get temperature(Celsius) from BME280 sensor
    temperaturebme = bme.temperature;

       Serial.print("Temperature: ");
    Serial.println(temperaturebme);

    // get humidity from BME280 sensor
    humidity = bme.humidity;


    Serial.println("Humidity: ");
    Serial.println(humidity);
   
    Serial.println("Inside Temperature: ");
    Serial.println(tempDS);
    
    payload = "{\"InsideTemp\":" + String(tempDS) + ",\"OutsideTemp\":" + String(temperaturebme) + ",\"DoorState\":" + "\"" +  door_state + "\"" + "}";
    Serial.println(payload);
    publishData(payload);
    lastMsg = now;
  }
}
