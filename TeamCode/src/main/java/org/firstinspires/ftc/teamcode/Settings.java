package org.firstinspires.ftc.teamcode;

import android.os.Bundle;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.android.dex.Code;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.json.JSONException;
import org.json.JSONObject;
import java.io.File;

/**
 * Created by SalineRobotics on 11/12/2017.
 */

public class Settings {
    public String settingName;
    public int setting1;
    public int setting2;

    private JSONObject jsonXSettings;


    public void init() {
        settingName = "AutonBlue.json";
        setting1=5;
        setting2=10;
    }

    public void newSettings() {
        jsonXSettings = new JSONObject();
    }

    public void SetSetting(String name, Double x) {
        try {
            jsonXSettings.put(name, x);
        }
        catch (JSONException e) {}
    }

    public void SetIntSetting(String name, Integer x) {
        try {
            jsonXSettings.put(name, x);
        }
        catch (JSONException e) {}
    }

    public void SaveSetting(String path) {
        File file = AppUtil.getInstance().getSettingsFile(path);
        ReadWriteFile.writeFile(file, jsonXSettings.toString());
    }


    public void ReadSettings() {
        File file = AppUtil.getInstance().getSettingsFile(settingName);
        String jsonString = ReadWriteFile.readFile(file);
        Deserialize(jsonString);
    }

    public void WriteSettings(String mydata) {
        File file = AppUtil.getInstance().getSettingsFile(settingName);
        if (mydata == "") {
            ReadWriteFile.writeFile(file, Serialize());
        } else {
            ReadWriteFile.writeFile(file, mydata);
        }
    }

    public String Serialize() {
        JSONObject jsonSettings = new JSONObject();
        try {
            jsonSettings.put("setting1" , setting1);
            jsonSettings.put("setting2", setting2);
            jsonSettings.put("setting123", 456);
        }
        catch (JSONException e) {
            //return default settings?
        }

        return jsonSettings.toString();
    }

    public void Deserialize(String settingString) {

        try {
            JSONObject jsonSettings = new JSONObject(settingString);
            setting1 = jsonSettings.getInt("setting2");
            setting2 = jsonSettings.getInt("setting1");
        }
        catch (JSONException e) {
            //return default settings?
        }
    }
}
