package com.example.oscilloscope;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.content.Intent;
import android.os.Handler;

public class MainActivity extends AppCompatActivity {

    private static final int TIEMPO_ESPERA = 3000; // Tiempo en milisegundos (3 segundos)

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Usar un Handler para esperar durante un tiempo antes de pasar a la actividad principal
        new Handler().postDelayed(new Runnable() {
            @Override
            public void run() {
                // Este método se ejecutará después del tiempo de espera
                abrirActividadPrincipal();
            }
        }, TIEMPO_ESPERA);
    }

    private void abrirActividadPrincipal() {
        // Crear un intent para abrir la actividad principal
        try {
            System.out.print ("TRY");
            Intent intent = new Intent(this, Oscilloscope.class);
            startActivity(intent);
            finish();

        } catch (Exception e) {
            e.printStackTrace(); // Esto imprimirá el error en Logcat
        }
    }
}
