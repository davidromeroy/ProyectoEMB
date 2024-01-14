package com.example.oscilloscope;

import java.io.IOException;
import java.io.OutputStream;
import java.net.Socket;

public class WifiClient {
    private String ipAddress;
    private int port;

    public WifiClient(String ipAddress, int port) {
        this.ipAddress = ipAddress;
        this.port = port;
    }

    public void sendString(String data) {
        new Thread(() -> {
            try (Socket socket = new Socket(ipAddress, port);
                 OutputStream outputStream = socket.getOutputStream()) {
                String dataWithNewLine = data + "\n"; // Añadir un salto de línea al final
                outputStream.write(dataWithNewLine.getBytes());
                outputStream.flush();
            } catch (IOException e) {
                e.printStackTrace();
                // Manejar errores aquí
            }
        }).start();
    }
}

