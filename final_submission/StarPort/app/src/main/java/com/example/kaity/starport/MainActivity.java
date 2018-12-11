package com.example.kaity.starport;

import android.app.Activity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.Toast;

/* Main Activity UI Thread that defines button onClick behavior. */
public class MainActivity extends Activity {
    /* Initialize Button placeholders from UI */
    Button warm;
    Button cool;
    Button idle;
    Button power_on;
    Button power_off;
    Button energy_on;
    Button energy_off;
    Button lock_on;
    Button lock_off;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        /* Get Buttons by ID */
        warm = findViewById(R.id.warm_button);
        cool = findViewById(R.id.cool_button);
        idle = findViewById(R.id.idle_button);
        power_on = findViewById(R.id.power_on);
        power_off = findViewById(R.id.power_off);
        energy_on = findViewById(R.id.energy_on);
        energy_off = findViewById(R.id.energy_off);
        lock_on = findViewById(R.id.lock_on);
        lock_off = findViewById(R.id.lock_off);

        /* Warm mode selection */
        warm.setOnClickListener(new View.OnClickListener() {
            /* When clicked, send TCP packet for warm mode turned on. */
            @Override
            public void onClick(View view) {
                Toast.makeText(getApplicationContext(), "Warm mode on", Toast.LENGTH_LONG).show();
                String message = "W";
                /* send the message to the server */
                send(message);
            }
        });

        /* Cool mode selection */
        cool.setOnClickListener(new View.OnClickListener() {
            /* When clicked, send TCP packet for cool mode turned on. */
            @Override
            public void onClick(View view) {
                Toast.makeText(getApplicationContext(), "Cool mode on", Toast.LENGTH_LONG).show();
                String message = "C";
                /* send the message to the server */
                send(message);
            }
        });

        /* Idle mode selection */
        idle.setOnClickListener(new View.OnClickListener() {
            // When clicked, send TCP packet for idle mode turned on. */
            @Override
            public void onClick(View view) {
                Toast.makeText(getApplicationContext(), "Idle mode on", Toast.LENGTH_LONG).show();
                String message = "I";
                /* send the message to the server */
                send(message);
            }
        });

        /* Power on command */
        power_on.setOnClickListener(new View.OnClickListener() {
            /* When clicked, send TCP packet for LEDs to be turned on. */
            @Override
            public void onClick(View view) {
                Toast.makeText(getApplicationContext(), "Power on", Toast.LENGTH_LONG).show();
                String message = "P";
                /* send the message to the server */
                send(message);
            }
        });

        /* Power off command */
        power_off.setOnClickListener(new View.OnClickListener() {
            /* When clicked, send TCP packet for LEDs to be turned off. */
            @Override
            public void onClick(View view) {
                Toast.makeText(getApplicationContext(), "Power off", Toast.LENGTH_LONG).show();
                String message = "Q";
                /* send the message to the server */
                send(message);
            }
        });

        /* Energy saver mode on command */
        energy_on.setOnClickListener(new View.OnClickListener() {
            /* When clicked, send TCP packet for Energy Saver mode to be turned on. */
            @Override
            public void onClick(View view) {
                Toast.makeText(getApplicationContext(), "Energy saver on", Toast.LENGTH_LONG).show();
                String message = "E";
                /* send the message to the server */
                send(message);
            }
        });
        
        /* Energy saver mode off command */
        energy_off.setOnClickListener(new View.OnClickListener() {
            /* When clicked, send TCP packet for Energy Saver mode to be turned off. */
            @Override
            public void onClick(View view) {
                Toast.makeText(getApplicationContext(), "Energy saver off", Toast.LENGTH_LONG).show();
                String message = "F";
                /* send the message to the server */
                send(message);
            }
        });

        /* Lock out alarm mode on command */
        lock_on.setOnClickListener(new View.OnClickListener() {
            /* When clicked, send TCP packet for Lockout alarm mode to be turned on. */
            @Override
            public void onClick(View view) {
                Toast.makeText(getApplicationContext(), "Lockout on", Toast.LENGTH_LONG).show();
                String message = "L";
                /* send the message to the server */
                send(message);
            }
        });

        /* Lock out alarm mode off command */
        lock_off.setOnClickListener(new View.OnClickListener() {
            /* When clicked, send TCP packet for Lockout alarm mode to be turned off. */
            @Override
            public void onClick(View view) {
                Toast.makeText(getApplicationContext(), "Lockout off", Toast.LENGTH_LONG).show();
                String message = "M";
                /* send the message to the server */
                send(message);
            }
        });
    }

    /* Send TCP packet to server by calling MessageSender class, an asynchronous task. */
    public void send(String message)
    {
        MessageSender messageSender = new MessageSender();
        messageSender.execute(message);
    }
}