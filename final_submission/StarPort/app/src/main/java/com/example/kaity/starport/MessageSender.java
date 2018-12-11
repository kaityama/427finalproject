package com.example.kaity.starport;

import android.os.AsyncTask;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.PrintWriter;
import java.net.Socket;

/* Asynchronous task that sends messages on a TCP socket
 * separate from the UI thread. */
public class MessageSender extends AsyncTask<String, String, Void> {
    Socket s;
    PrintWriter pw;
    BufferedReader in;
    public static final String SERVER_IP = "172.20.10.4"; // server IP address
    public static final int SERVER_PORT = 4210; // select a port

    @Override
    protected Void doInBackground(String... voids) {
        String message = voids[0];

        /*  Opens Socket and writes to it */
        try {
            s = new Socket(SERVER_IP, SERVER_PORT);
            pw = new PrintWriter(s.getOutputStream());
            pw.write(message);
            pw.flush();
            pw.close();
            s.close();
        } catch (IOException ie) {
            ie.printStackTrace();
        }
        return null;
    }
}