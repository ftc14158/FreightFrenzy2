package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Hub+Warehouse-Delayed15sec")

public class DelayHubWarehouse extends VisionAutonomous {

    public DelayHubWarehouse() {
        super();
        startDelay = 15000;
    }

}
