package org.firstinspires.ftc.teamcode;

import android.os.Bundle;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.json.JSONException;
import org.json.JSONObject;
import java.io.File;

/**
 * Created by SalineRobotics on 11/12/2017.
 */

public class Settings {

    private JSONObject jsonXSettings;

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

    public void ReadSettings(String path) {
        File file = AppUtil.getInstance().getSettingsFile(path);
        try {
            jsonXSettings = new JSONObject(ReadWriteFile.readFile(file));
        }
        catch (JSONException e) {
            jsonXSettings = new JSONObject();
        }
    }

    public Double GetSetting(String name) {
        try {
            return jsonXSettings.getDouble(name);
        }
        catch (JSONException e) {
            return 0.0;
        }
    }

    public int GetIntSetting(String name) {
        try {
            return jsonXSettings.getInt(name);
        }
        catch (JSONException e) {
            return 0;
        }
    }

}
