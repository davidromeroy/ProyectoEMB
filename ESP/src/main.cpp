#include <Arduino.h>
#include <driver/adc.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <NTPClient.h>


// WiFi
#define WIFI_SSID ""        // Cambiar por la red WiFi a conectarse
#define WIFI_PASS ""         // Cambiar por la clave de la red WiFi a conectarse


//Ubidots
#define TOKEN ""             // Coloca tu TOKEN aquí de Ubidots
#define MQTT_CLIENT_NAME "mymqttclient"
#define DEVICE_LABEL "Oscilloscope"     // Coloca la etiqueta de tu dispositivo aquí
#define VARIABLE_LABEL "signal"         // Etiqueta de la variable de voltaje del dispositivo

WiFiServer server(80);

WiFiClient ubidots;
PubSubClient client(ubidots);


char mqttBroker[] = "industrial.api.ubidots.com";
char payload[10000];
char topic[150];
//Space to store values to send
char str_sensor[10];
char str_millis[20];
double epochseconds = 0;
double epochmilliseconds = 0;
double current_millis = 0;
double current_millis_at_sensordata = 0;
double timestampp = 0;
int j = 0;

float amp;
float cicleDuty;
bool enviarDatos = false;
float sensor;


// Configuración de las señales
const int dacPin = 25; // Pin DAC (GPIO 25 en este ejemplo)
const int samplesPerCycle = 100; // Número de muestras por ciclo
unsigned long lastSampleTime = 0;
const float freq = 0.5; // Frecuencia de la señal senoidal en Hz
const unsigned long period = 1000000 / freq; // Periodo en microsegundos

int sampleIndex = 0; //S
int sampleValue = 0;//T
bool increasing = true;//T

char tipoOnda = 'S' ; 

// Configuración del "osciloscopio"
const int adcPin = 34; // Pin para la lectura analógica (A2)
const int adcResolution = 12; // Resolución del ADC

// Pin del potenciómetro
const int potentiometerPin = 35;


/***
 * Funciones generadoras de señal
 ****/

//void senoTask(void *pvParameters) {
void senoTask() {
  //while (1) {
    unsigned long currentTime = micros();
    if (currentTime - lastSampleTime >= period / samplesPerCycle) {
      lastSampleTime = currentTime;

      float adjustedPhase;
      if (sampleIndex < samplesPerCycle * cicleDuty) {
        // Fase ajustada para el tiempo de subida
        adjustedPhase = (sampleIndex / (samplesPerCycle * cicleDuty
)) * PI;
      } else {
        // Fase ajustada para el tiempo de bajada
        adjustedPhase = PI + ((sampleIndex - samplesPerCycle * cicleDuty
) / (samplesPerCycle * (1 - cicleDuty
))) * PI;
      }

      float senoValue = (sin(adjustedPhase) + 1) * round(amp/2);
      dacWrite(dacPin, int(senoValue));
      sampleIndex = (sampleIndex + 1) % samplesPerCycle;
    }
    //vTaskDelay(1); // Cede el control
  //}
}

//void triangleTask(void *pvParameters) {
void triangleTask() {
  //while (1) {
    unsigned long currentTime = micros();
    if (currentTime - lastSampleTime >= period / samplesPerCycle) {
      lastSampleTime = currentTime;

      // Realiza los cálculos en punto flotante
      float increment = float(amp) / (samplesPerCycle * (increasing ? cicleDuty : (1 - cicleDuty)));

      // Actualiza el valor de la señal triangular
      if (increasing) {
        sampleValue += round(increment); // Usa round para evitar perder la precisión antes de convertir a entero
        if (sampleValue >= amp) {
          sampleValue = amp;
          increasing = false;
        }
      } else {
        sampleValue -= round(increment);
        if (sampleValue <= 0) {
          sampleValue = 0;
          increasing = true;
        }
      }

      dacWrite(dacPin, sampleValue);
    }
    //vTaskDelay(1); // Cede el control
  //}
}

//void squareWaveTask(void *pvParameters) {
void squareWaveTask() {
  //while (1) {
    unsigned long currentTime = micros();
    if (currentTime - lastSampleTime >= period / samplesPerCycle) {
      lastSampleTime = currentTime;

      int squareValue;
      if (sampleIndex < samplesPerCycle * cicleDuty) {
        // Tiempo de subida: la señal está en alto
        squareValue = amp; // Valor máximo para DAC, ajustar según la resolución de tu DAC
      } else {
        // Tiempo de bajada: la señal está en bajo
        squareValue = 0;
      }

      dacWrite(dacPin, squareValue);
      sampleIndex = (sampleIndex + 1) % samplesPerCycle;
    }
    //vTaskDelay(1); // Cede el control
  //}
}

// Tarea para actuar como osciloscopio

//void readVoltage(void *pvParameters) {
float readVoltage() {
  //while (1) {
    float adcValue34 = adc1_get_raw(ADC1_CHANNEL_6); // Leer valor ADC del pin34
    //Serial.println(adcValue34); // Enviar valor por puerto serial
    //Serial.println(',');
    return adcValue34;
    //vTaskDelay(pdMS_TO_TICKS(1)); // 1ms delay para una mejor captura
  //}
}

void generateSignalTask(){
//void generateSignalTask(void *pvParameters){
  
  switch (tipoOnda) {
    case 'T':
        // Acción para la onda Triangular
        triangleTask();
        break;
    case 'S':
        // Acción para onda Sinusoidal
        senoTask();
        break;
    case 'P':
        // Acción para onda PWM
        squareWaveTask();
        break;
    default:
        // Acción por defecto si se recibe otra cosa
        Serial.println("Letra no reconocida");
        break;
  }
}


/***
 * Primer Ciclo
 ****/

void cicloUnoTask(void *pvParameters){
  while (1) {
    generateSignalTask();
    sensor = (readVoltage()*2)/(819);
    vTaskDelay(pdMS_TO_TICKS(1));
  }

}

/***
 * Auxiliar Functions
 ****/

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");

void callback(char* topic, byte* payload, unsigned int length){
  char p[length + 1];
  memcpy(p, payload, length);
  p[length] = NULL;
  Serial.write(payload,length);
  Serial.println(topic);
}

void reconnect(){
  //insistir hasta que nos volvamos a conectar
  while (!client.connected()){
    Serial.println("Attempting MQTT connection..."); //Intenta conectarte
    if (client.connect(MQTT_CLIENT_NAME, TOKEN, "")){
      Serial.println("Connected"); //Si el cliente se conecta imprime conectado.
    } else{
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 2 seconds");
      //Espere 2 segundos antes de volver a intentarlo
      delay(2000);
    }
  }
}

void procesarMensaje(String mensaje) {
    // Divide el mensaje en dos partes usando la coma como separador
    int idx = mensaje.indexOf(';');  // Encuentra la posición de la coma

    // Extrae la amplitud y el ciclo de trabajo del mensaje
    String strAmp = mensaje.substring(0, idx);
    String strCicleDuty = mensaje.substring(idx + 1,-1);

    // Convierte los valores de String a float/int
    amp = strAmp.toFloat();
    cicleDuty = strCicleDuty.toFloat();
    // Imprime los valores para verificar
    Serial.print("Amplitud: ");
    Serial.println(amp);
    Serial.print("Ciclo de trabajo: ");
    Serial.println(cicleDuty);
}

void receivData() {
    WiFiClient client = server.available(); // Escucha a los clientes entrantes
    if (client) {
        //Serial.println("Nuevo Cliente.");
        String currentLine = ""; // String para guardar los datos desde el cliente
        while (true) {
            if (client.available()) {
                char c = client.read(); // lee un byte
                if (c == '\n') { // si el byte es un salto de línea, indica el final de un mensaje
                    if (currentLine.length() == 1){
                        // Lee el primer byte del cliente
                        tipoOnda = currentLine[0];
                    } else if(currentLine=="FALSE"){
                        enviarDatos = false;
                      } else if(currentLine=="TRUE"){
                        enviarDatos = true;
                      } else{    
                      //Serial.println("Mensaje completo recibido: " + currentLine);
                      procesarMensaje(currentLine); // Procesa el mensaje completo
                      }
                    currentLine = ""; // Limpia la línea actual para el próximo mensaje
                    break;
                } else if (c != '\n') {
                    currentLine += c; // añade al final de la línea actual
                    //delay(100);
                }
            }
        }
        if (currentLine.length() > 0) { // Verifica si hay un mensaje sin terminar cuando el cliente se desconecta
            Serial.println("Mensaje incompleto recibido: " + currentLine);
            //procesarMensaje(currentLine);
        }
        client.stop(); // Cierra la conexión
        //Serial.println("Cliente Desconectado.");
    }
}

/***
 * Segundo Ciclo
 ****/

void cicloDosTask(void *pvParameters){
  while (1) {
      receivData();
  
  if (!client.connected()) {
    reconnect();
    j = 0;
  }
  //sprintf(payload, "%s", "{\"ECG_Sensor_data\": [{\"value\":1234, \"timestamp\": 1595972075},{\"value\":1111, \"timestamp\": 1595971075},{\"value\":2222, \"timestamp\": 1595970075}]}");
  client.loop();
  if (enviarDatos) {
      j = j + 1;
      //Serial.print("j=");
      //Serial.println(j);
      sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
      sprintf(payload, "%s", ""); // Cleans the payload
      sprintf(payload, "{\"%s\": [", VARIABLE_LABEL); // Adds the variable label
      for (int i = 1; i <= 3; i++)
      {
        dtostrf(sensor, 4, 2, str_sensor);
        sprintf(payload, "%s{\"value\":", payload); // Adds the value
        sprintf(payload, "%s %s,", payload, str_sensor); // Adds the value
        current_millis_at_sensordata = millis();
        timestampp = epochmilliseconds + (current_millis_at_sensordata - current_millis);
        dtostrf(timestampp, 10, 0, str_millis);
        sprintf(payload, "%s \"timestamp\": %s},", payload, str_millis); // Adds the value
        //delay(100);
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      //Serial.println(sensor);
      dtostrf(sensor, 4, 2, str_sensor);
      current_millis_at_sensordata = millis();
      timestampp = epochmilliseconds + (current_millis_at_sensordata - current_millis);
      dtostrf(timestampp, 10, 0, str_millis);
      sprintf(payload, "%s{\"value\":%s, \"timestamp\": %s}]}", payload, str_sensor, str_millis);
      Serial.println("Publishing data to Ubidots Cloud");
      client.publish(topic, payload);
      Serial.println(payload);
  }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

/***
 * Principal Functions
 ****/

void setup() {
  Serial.begin(115200);
  Serial.println();
  WiFi.disconnect(true);
  // Iniciar conexión WiFi
  Serial.println("Conectando a la red WiFi...");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  
  timeClient.begin();
  client.setServer(mqttBroker, 1883);
  client.setCallback(callback);
  timeClient.update();
  epochseconds = timeClient.getEpochTime();
  epochmilliseconds = epochseconds * 1000;
  Serial.print("epochmilliseconds=");
  Serial.println(epochmilliseconds);
  current_millis = millis();
  Serial.print("current_millis=");
  Serial.println(current_millis);
  server.begin();

  // Configuración del ADC
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);

  // Crear tarea de generación de señal y osciloscopio en el núcleo 0
  xTaskCreatePinnedToCore(cicloUnoTask, "cicloUno Task", 10000, NULL, 1, NULL, 0);

  // Crear tarea de  conectividd wifi en el núcleo 1
  xTaskCreatePinnedToCore(cicloDosTask, "cicloUno Task", 10000, NULL, 1, NULL, 1);

}
void loop() {

}
