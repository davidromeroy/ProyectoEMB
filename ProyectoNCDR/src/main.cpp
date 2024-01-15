//#include <Arduino.h>
//#include <driver/adc.h>

// Configuración de la señal PWM
//const int pwmPin = 25; // Pin para la señal PWM
//const int pwmFreq = 6; // Frecuencia de la señal PWM
//const int pwmResolution = 8; // Resolución del PWM (8 bits)
//int dutyCycle = 128;

// Configuración de la señal senoidal
//const int dacPin = 25; // Pin DAC (GPIO 25 en este ejemplo)
//const int senoFreq = 1; // Frecuencia de la señal senoidal en Hz
//const unsigned long senoPeriod = 1000000 / senoFreq; // Periodo en microsegundos
//const int samplesPerCycle = 100; // Número de muestras por ciclo
//unsigned long lastSampleTime = 0;
//int sampleIndex = 0;

// Configuración de la señal triangular
//const int dacPin = 25; // Pin DAC (GPIO 25 en este ejemplo)
//const int triFreq = 5; // Frecuencia de la señal triangular en Hz
//const unsigned long triPeriod = 1000000 / triFreq; // Periodo en microsegundos
//const int samplesPerCycle = 100; // Número de muestras por ciclo
//unsigned long lastSampleTime = 0;
//int sampleValue = 0;
//bool increasing = true;

// Configuración de la señal PWM
//const int dacPin = 25; // Pin DAC del ESP32
//float amplitud = 0.2; // Amplitud como fracción de 3.3V (1.0 es igual a 3.3V)
//int frecuencia = 1; // Frecuencia en Hz
//float dutyCycle = 0.5; // Ciclo de trabajo (0.0 a 1.0)

// Configuración del "osciloscopio"
//const int adcPin = 34; // Pin para la lectura analógica (A2)
//const int adcResolution = 12; // Resolución del ADC

// Tarea para generar señal PWM
//void pwmTask(void *pvParameters) {
//  while (1) {
//  ledcWrite(0, dutyCycle);
//  vTaskDelay(1); // Cede el control cada 1 tick (ajusta según sea necesario)
//  }
//}

//void senoTask(void *pvParameters) {
//  while (1) {
//    unsigned long currentTime = micros();
//    if (currentTime - lastSampleTime >= senoPeriod / samplesPerCycle) {
//      lastSampleTime = currentTime;
//      float senoValue = (sin(2 * PI * sampleIndex / samplesPerCycle) + 1) * 127.5; // Valor senoidal
//      dacWrite(dacPin, int(senoValue));
//      sampleIndex = (sampleIndex + 1) % samplesPerCycle;
//    }
//    vTaskDelay(1); // Cede el control
//  }
//}

//void triangleTask(void *pvParameters) {
//  while (1) {
//    unsigned long currentTime = micros();
//    if (currentTime - lastSampleTime >= triPeriod / samplesPerCycle) {
//      lastSampleTime = currentTime;
//
//      // Actualiza el valor de la señal triangular
//      if (increasing) {
//        sampleValue += 255 / (samplesPerCycle / 2); // Aumenta
//        if (sampleValue >= 255) {
//          sampleValue = 255;
//          increasing = false;
//        }
//      } else {
//        sampleValue -= 255 / (samplesPerCycle / 2); // Disminuye
//        if (sampleValue <= 0) {
//          sampleValue = 0;
//          increasing = true;
//        }
//      }
//
//      dacWrite(dacPin, sampleValue);
//    }
//    vTaskDelay(1); // Cede el control
//  }
//}

//void pwmTask(void *pvParameters) {
//  while (1) {
//    int periodo = 1000 / frecuencia; // Periodo en milisegundos
//    int tiempoAlto = periodo * dutyCycle; // Tiempo en alto basado en el ciclo de trabajo
//    int tiempoBajo = periodo - tiempoAlto; // Tiempo en bajo
//    int voltajeMaximo = 255 * amplitud; // Valor máximo de DAC para la amplitud deseada
//
//    // Generar una señal cuadrada
//    dacWrite(dacPin, voltajeMaximo);
//    vTaskDelay(pdMS_TO_TICKS(tiempoAlto));
//    dacWrite(dacPin, 0);
//    vTaskDelay(pdMS_TO_TICKS(tiempoBajo));
//  }
//}

// Tarea para actuar como osciloscopio
//void oscilloscopeTask(void *pvParameters) {
//  while (1) {
//    int adcValue34R = adc1_get_raw(ADC1_CHANNEL_6); // Leer valor ADC del pin34
//    Serial.println(adcValue34R);
//    //float adcValue34 = (adcValue34R/4095)*3.33;
//    //Serial.println(adcValue34); // Enviar valor por puerto serial
//    Serial.println(',');
//    vTaskDelay(pdMS_TO_TICKS(1)); // 1ms delay para una mejor captura
//  }
//}

//void setup() {
  //Serial.begin(115200);

  // Configuración del PWM
  //ledcSetup(0, pwmFreq, pwmResolution);
  //ledcAttachPin(pwmPin, 0);

  // Configuración del ADC
  //adc1_config_width(ADC_WIDTH_BIT_12);
  //adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);

  // Crear tarea de PWM en el núcleo 0
  //xTaskCreatePinnedToCore(pwmTask, "PWM Task", 10000, NULL, 1, NULL, 0);
  //xTaskCreatePinnedToCore(senoTask, "Seno Task", 10000, NULL, 1, NULL, 0);
  //xTaskCreatePinnedToCore(triangleTask, "Triangle Task", 10000, NULL, 1, NULL, 0);

  // Crear tarea de osciloscopio en el núcleo 1
  //xTaskCreatePinnedToCore(oscilloscopeTask, "Oscilloscope Task", 10000, NULL, 1, NULL, 1);
//}

//void loop() {
  // El loop principal puede quedar vacío
//}

#include <WiFi.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <NTPClient.h>


// WiFi
#define WIFI_SSID ""   //Cambiar
#define WIFI_PASS ""  //Cambiar

//Ubidots
//#define TOKEN "BBUS-uj2EJdJpGj8qNcGVd9AEMQDl26yorC"             // Coloca tu TOKEN aquí
#define TOKEN "BBUS-ub7Bxhi4yjlnfhTcr1Xs4Nh1Kc1PZW"       //New
#define MQTT_CLIENT_NAME "mymqttclient"
#define DEVICE_LABEL "Oscilloscope"  // Coloca la etiqueta de tu dispositivo aquí
#define VARIABLE_LABEL "signal"  // Etiqueta de la variable de voltaje

//#define VARIABLE_LABEL_TIME "time"        // Etiqueta de la variable de tiempo
#define SENSORPIN 34

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
int cicleDuty;
bool enviarDatos = false;



/*******
 * Auxiliar Functions
 ********/

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

float readVoltage() {
  // Simula la lectura de un sensor de voltaje, reemplaza esto con tu lógica real
  return random(0, 500) / 100.0;  // Valor aleatorio entre 0 y 5
}

void receivData() {
    WiFiClient client = server.available(); // Escucha a los clientes entrantes
    if (client) {
        Serial.println("Nuevo Cliente.");
        String currentLine = ""; // String para guardar los datos desde el cliente
        while (true) {
            if (client.available()) {
                char c = client.read(); // lee un byte
                //Serial.write(c);       // imprime ese byte en el monitor serial

                if (c == '\n') { // si el byte es un salto de línea, indica el final de un mensaje
                    if (currentLine.length() == 1){
                        // Lee el primer byte del cliente
                        char tipoOnda = currentLine[0];
                        Serial.print("Tipo de onda: ");              
                        switch (tipoOnda) {
                          case 'T':
                              // Acción para la onda Triangular
                              Serial.println("Onda Triangular");
                              break;
                          case 'S':
                              // Acción para onda Sinusoidal
                              Serial.println("Onda Sinusoidal");
                              break;
                          case 'P':
                              // Acción para onda PWM
                              Serial.println("Onda PWM");
                              break;
                          default:
                              // Acción por defecto si se recibe otra cosa
                              Serial.println("Letra no reconocida");
                              break;
                        }
                    } else{    
                      Serial.println("Mensaje completo recibido: " + currentLine);
                      procesarMensaje(currentLine); // Procesa el mensaje completo
                      if(currentLine=="FALSE"){
                        enviarDatos = false;
                      } else if(currentLine=="TRUE"){
                        enviarDatos = true;
                      }
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
            procesarMensaje(currentLine);
        }
        client.stop(); // Cierra la conexión
        Serial.println("Cliente Desconectado.");
    }
}


void procesarMensaje(String mensaje) {
    // Divide el mensaje en dos partes usando la coma como separador
    int idx = mensaje.indexOf(',');  // Encuentra la posición de la coma

    // Extrae la amplitud y el ciclo de trabajo del mensaje
    String strAmp = mensaje.substring(0, idx);
    String strCicleDuty = mensaje.substring(idx + 1);

    // Convierte los valores de String a float/int
    amp = strAmp.toFloat();
    cicleDuty = strCicleDuty.toInt();

    // Imprime los valores para verificar
    Serial.print("Amplitud: ");
    Serial.println(amp);
    Serial.print("Ciclo de trabajo: ");
    Serial.println(cicleDuty);
}


/*******
 * Principal Functions
 ********/
 
void setup() {
  Serial.begin(115200);
  // Assign the pin as INPUT
  pinMode(SENSORPIN, INPUT);
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

}
void loop() {
  receivData();
  
  if (!client.connected()) {
    reconnect();
    j = 0;
  }
  //sprintf(payload, "%s", "{\"ECG_Sensor_data\": [{\"value\":1234, \"timestamp\": 1595972075},{\"value\":1111, \"timestamp\": 1595971075},{\"value\":2222, \"timestamp\": 1595970075}]}");
  client.loop();
  if (enviarDatos) {
      j = j + 1;
      Serial.print("j=");
      Serial.println(j);
      sprintf(topic, "%s%s", "/v1.6/devices/", DEVICE_LABEL);
      sprintf(payload, "%s", ""); // Cleans the payload
      sprintf(payload, "{\"%s\": [", VARIABLE_LABEL); // Adds the variable label
      for (int i = 1; i <= 3; i++)
      {
        float sensor = readVoltage();
        dtostrf(sensor, 4, 2, str_sensor);
        sprintf(payload, "%s{\"value\":", payload); // Adds the value
        sprintf(payload, "%s %s,", payload, str_sensor); // Adds the value
        current_millis_at_sensordata = millis();
        timestampp = epochmilliseconds + (current_millis_at_sensordata - current_millis);
        dtostrf(timestampp, 10, 0, str_millis);
        sprintf(payload, "%s \"timestamp\": %s},", payload, str_millis); // Adds the value
        delay(150);
      }
      float sensor = readVoltage();
      dtostrf(sensor, 4, 2, str_sensor);
      current_millis_at_sensordata = millis();
      timestampp = epochmilliseconds + (current_millis_at_sensordata - current_millis);
      dtostrf(timestampp, 10, 0, str_millis);
      sprintf(payload, "%s{\"value\":%s, \"timestamp\": %s}]}", payload, str_sensor, str_millis);
      Serial.println("Publishing data to Ubidots Cloud");
      client.publish(topic, payload);
      Serial.println(payload);
  }
}

