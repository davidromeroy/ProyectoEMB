package com.example.oscilloscope;

import androidx.appcompat.app.AppCompatActivity;

import android.annotation.SuppressLint;
import android.os.Bundle;
import android.view.Window;
import android.view.WindowManager;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.Button;
import android.widget.EditText;
import android.view.View;
import android.widget.Toast;

import java.io.IOException;

public class Oscilloscope extends AppCompatActivity {

    private UbidotsClient ubidotsClient;
    private WifiClient wifiClient;
    private Button triangleButton;
    private Button sinButton;
    private Button pwmButton;
    private Button stopButton;
    private EditText editTextAmp;
    private EditText editTextCicleDuty;
    private EditText editTextFrec;
    private WebView webView;


    @SuppressLint("WrongViewCast")
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        getWindow().setFlags(
                WindowManager.LayoutParams.FLAG_FULLSCREEN,
                WindowManager.LayoutParams.FLAG_FULLSCREEN);

        setContentView(R.layout.activity_oscilloscope);
        triangleButton = findViewById(R.id.triangleButton);
        sinButton = findViewById(R.id.sinButton);
        pwmButton = findViewById(R.id.pwmButton);
        stopButton = findViewById(R.id.stopButton);
        editTextAmp = findViewById(R.id.editTextAmp);
        editTextCicleDuty = findViewById(R.id.editTextCicleDuty);
        //editTextFrec = findViewById(R.id.editTextFrec);

        webView = (WebView) findViewById(R.id.webview);
        webView.setWebViewClient(new WebViewClient()); // Asegura que los enlaces se abran dentro del WebView
        webView.getSettings().setJavaScriptEnabled(true); // Habilita JavaScript si es necesario

        // URL del widget de Ubidots
        String url = "https://stem.ubidots.com/app/dashboards/public/widget/nHR1alzUcLL4VCx4vjULTflGRvf-n4idV4Zs-_453mc?from=1705825757448&to=1705912157448&datePicker=true";       //Cambiar por el link publico del respectivo widget de la gráfica de Ubidots
        webView.loadUrl(url);

        wifiClient = new WifiClient("192.168.1.2", 80); // Reemplaza con la IP y puerto del ESP32

        triangleButton.setOnClickListener(v -> {
            wifiClient.sendString("T");
            if (startAction()) {
                // Desactivar los otros botones solo si startAction devuelve true
                sinButton.setEnabled(false);
                pwmButton.setEnabled(false);
            }
        });
        sinButton.setOnClickListener(v -> {
            wifiClient.sendString("S");
            if (startAction()) {
                // Desactivar los otros botones solo si startAction devuelve true
                triangleButton.setEnabled(false);
                pwmButton.setEnabled(false);
            }
        });
        pwmButton.setOnClickListener(v -> {
            wifiClient.sendString("P");
            if (startAction()) {
                // Desactivar los otros botones solo si startAction devuelve true
                triangleButton.setEnabled(false);
                sinButton.setEnabled(false);
            }
        });
        stopButton.setOnClickListener(v -> {
            wifiClient.sendString("FALSE");
            Toast.makeText(Oscilloscope.this, "STOP", Toast.LENGTH_SHORT).show();

            // Activa los otros botones
            triangleButton.setEnabled(true);
            sinButton.setEnabled(true);
            pwmButton.setEnabled(true);
        });

        // Inicializa el cliente de Ubidots
        String apiKey = "BBUS-uj2EJdJpGj8qNcGVd9AEMQDl26yorC"; // Reemplaza con tu API Key real
        ubidotsClient = new UbidotsClient(apiKey);
    }

    // Método para obtener datos de Ubidots
    private void obtenerDatosDeUbidots() {
        String deviceId = "Oscilloscope"; // Reemplaza con el ID de tu dispositivo
        String variableId = "signal"; // Reemplaza con el ID de tu variable

        new Thread(() -> {
            try {
                String value = ubidotsClient.getVariableValue(deviceId, variableId);
                // Aquí actualizas tu UI o lógica con el valor obtenido
                // Recuerda que cualquier actualización de la UI debe hacerse en el hilo principal



            } catch (IOException e) {
                e.printStackTrace();
                // Maneja el error aquí
            }
        }).start();
    }

    private boolean startAction() {
        // Obtener los valores ingresados por el usuario
        String ampValueStr = editTextAmp.getText().toString();
        String cicleDutyValueStr = editTextCicleDuty.getText().toString();


        // Verificar si alguno de los EditText está vacío
        if (ampValueStr.isEmpty() || cicleDutyValueStr.isEmpty()) {
            // Mostrar un mensaje de error o tomar alguna acción
            Toast.makeText(Oscilloscope.this, "Por favor, rellena todos los campos.", Toast.LENGTH_SHORT).show();
            return false; // Indica que la acción no debe continuar
        }

        try {
            float ampValue = (float) ((Float.parseFloat(ampValueStr))*(25.5));
            float cicleDutyValue = Float.parseFloat(cicleDutyValueStr)/100;

            // Verificar si los valores están en los rangos especificados
            if (ampValue < 0 || ampValue > 255) {
                Toast.makeText(Oscilloscope.this, "El valor de amplitud debe estar entre 0 y 10.", Toast.LENGTH_SHORT).show();
                return false;
            }

            if (cicleDutyValue < 0 || cicleDutyValue > 1) {
                Toast.makeText(Oscilloscope.this, "El valor del ciclo de trabajo debe estar entre 0 y 100.", Toast.LENGTH_SHORT).show();
                return false;
            }
            Toast.makeText(Oscilloscope.this, "START", Toast.LENGTH_SHORT).show();
            // Si los valores son válidos, continúa con el envío de los datos
            wifiClient.sendString("TRUE");
            String dataToSend = ampValue + ";" + cicleDutyValue;
            wifiClient.sendString(dataToSend);

            obtenerDatosDeUbidots();
            return true; // Indica que la acción debe continuar
        } catch (NumberFormatException e) {
            Toast.makeText(Oscilloscope.this, "Por favor, ingresa valores numéricos válidos.", Toast.LENGTH_SHORT).show();
            return false;
        }
    }
}
