package org.firstinspires.ftc.teamcode;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.view.LayoutInflater;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

public class ConfigDialog {
    private FtcRobotControllerActivity activity;

    public ConfigDialog(Context appContext) {
       activity = (FtcRobotControllerActivity) appContext;
    }

    public void show(){
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                LayoutInflater inflater = activity.getLayoutInflater();
                View alertView = inflater.inflate(R.layout.auto_config_dialog, null);

                AlertDialog a = new AlertDialog.Builder(activity)
                        .setTitle("config")
                        .setView(alertView)
                        .create();
                a.show();
            }
        });
    }
}
