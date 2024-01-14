package com.example.oscilloscope;

import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.Response;

import java.io.IOException;

public class UbidotsClient {
    private final OkHttpClient httpClient = new OkHttpClient();
    private final String apiKey;

    public UbidotsClient(String apiKey) {
        this.apiKey = apiKey;
    }

    public String getVariableValue(String deviceId, String variableId) throws IOException {
        Request request = new Request.Builder()
                .url("https://industrial.api.ubidots.com/api/v1.6/devices/" + deviceId + "/" + variableId + "/lv")
                .addHeader("X-Auth-Token", apiKey)
                .build();

        try (Response response = httpClient.newCall(request).execute()) {
            if (!response.isSuccessful()) throw new IOException("Unexpected code " + response);

            // Procesa y devuelve la respuesta
            return response.body().string();
        }
    }
}
