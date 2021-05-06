package edu.mit.chip.utils;

import java.util.HashMap;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Networking {
    private static Networking instance;

    private HashMap<String, NetworkTableEntry> readoutEntries;
    private HashMap<String, NetworkTableEntry> inputEntries;

    final NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    final NetworkTable readoutTable = networkTableInstance.getTable("rio_readout");
    final NetworkTable inputTable = networkTableInstance.getTable("rio_input");
    
    private Networking() {
        readoutEntries = new HashMap<String, NetworkTableEntry>();
        inputEntries = new HashMap<String, NetworkTableEntry>();
    }

    public void addReadouts(String... readoutEntries) {
        for (String entryName : readoutEntries) {
            this.readoutEntries.put(entryName, readoutTable.getEntry(entryName));
        }
    }

    public void addInputs(String... inputEntries) {
        for (String entryName : inputEntries) {
            this.inputEntries.put(entryName, inputTable.getEntry(entryName));
        }
    }

    public static Networking getInstance() {
        if (instance == null) {
            instance = new Networking();
        }

        return instance;
    }

    public void pushReadout(String entry, double value) {
        readoutEntries.get(entry).setDouble(value);
    }

    public void initInput(String entry, double value) {
        inputEntries.get(entry).setDouble(value);
    }

    public double pullInput(String entry, double defaultValue) {
        return inputEntries.get(entry).getDouble(defaultValue);
    }
}