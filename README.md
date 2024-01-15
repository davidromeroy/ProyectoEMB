# Osciloscopio y Generador de Funciones para ESP32

## Descripción
Este proyecto ofrece un osciloscopio y un generador de funciones diseñado para estudiantes principiantes en el campo de la electrónica. Utilizando una ESP32, proporcionamos herramientas accesibles para familiarizarse con instrumentos de laboratorio básicos, incorporando conceptos de telemática a través de la plataforma IoT Ubidots.

## Características
- **Osciloscopio:** Capaz de leer señales de voltaje positivas a baja frecuencia.
- **Generador de Funciones:** Produce señales variadas adecuadas para fines educativos.
- **Interfaz IoT:** Uso de Ubidots para la transmisión y visualización de datos.
- **Acondicionamiento de Señal:** Amplificación y reducción de señales para adaptarse al rango de voltaje de la ESP32.

## Limitaciones
- El osciloscopio está limitado a lecturas de voltajes positivos y a baja frecuencia.
- El rendimiento y la precisión están acordes con los componentes utilizados, adecuados para un entorno educativo.

## Requisitos Previos
- ESP32
- Amplificadores operacionales para acondicionamiento de señal
- Acceso a Ubidots o similar para la interfaz IoT
- Software y herramientas de desarrollo para ESP32 (por ejemplo, Arduino IDE)

## Configuración e Instalación
- Se debe descargar la aplicación EDUCOP que se utilizara como nexo entre UBIDOTS y el aparato en fisico para la manipulación, configuración y visualización de la graficas que se lean.
- Se debe conectar a la esp32 mediante wifi y subir el codigo correspondiente a este repositorio en la esp32.

## Contacto y autores
- Ney Coto - [ncoto@espol.edu.ec]
- David Romero -[daroyane@espol.edu.ec]
