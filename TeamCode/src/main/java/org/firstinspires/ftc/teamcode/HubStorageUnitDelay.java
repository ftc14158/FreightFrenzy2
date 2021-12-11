package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Hub+StorageUnit-Delayed15sec")

public class HubStorageUnitDelay extends HubStorageUnit {

    public HubStorageUnitDelay() {
        super();
        startDelay = 15000;
    }
}
