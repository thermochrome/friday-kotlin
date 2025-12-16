package org.firstinspires.ftc.teamcode.auton

import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.robot.Setup

@Autonomous(name = "Blue") @Suppress("unused")
class Main2: Setup() {
    private lateinit var follower: Follower
}