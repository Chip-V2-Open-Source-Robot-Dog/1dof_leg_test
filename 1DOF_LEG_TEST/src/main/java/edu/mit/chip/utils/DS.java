package edu.mit.chip.utils;

import java.net.InetSocketAddress;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.io.IOException;
import java.net.SocketException;

public class DS {
    private Thread thread;

    public void createEnablePacket(byte[] data, short count) {
        data[0] = (byte) (count >> 8);
        data[1] = (byte) count;
        data[2] = 0x01; // general tag
        data[3] = 0x04; // enable
        data[4] = 0x10; // request
        data[5] = 0x00; // station
    }

    public void start() {
        thread = new Thread(() -> {
            DatagramSocket socket;
            try {
                socket = new DatagramSocket();
            } catch(SocketException exception) {
                exception.printStackTrace();
                return;
            }
            
            InetSocketAddress address = new InetSocketAddress("127.0.0.1", 1110); // UDP port 1110: DS -> RoboRIO
            byte[] data = new byte[6];
            DatagramPacket packet = new DatagramPacket(data, 0, 6, address);
            
            // counters b/c for some reason 50 packets are required for the robot to actually enable
            short count = 0;
            int initCount = 0;

            while(!Thread.currentThread().isInterrupted()) {
                try {
                    Thread.sleep(20);
                    createEnablePacket(data, count++);
                    if(initCount < 50) {
                        initCount++;
                        data[3] = 0;
                    }
                    packet.setData(data);
                    socket.send(packet);
                } catch(InterruptedException exception) {
                    Thread.currentThread().interrupt();
                } catch(IOException exception) {
                    exception.printStackTrace();
                }
            }
            socket.close();
        });
        thread.setDaemon(true);
        thread.start();
    }

    public void stop() {
        if(thread == null) {
            return;
        }
        thread.interrupt();
        try {
            thread.join(1000);
        } catch(InterruptedException exception) {
            exception.printStackTrace();
        }
    }
}