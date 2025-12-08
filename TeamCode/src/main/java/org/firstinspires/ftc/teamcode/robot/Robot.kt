package org.firstinspires.ftc.teamcode.robot

import com.arcrobotics.ftclib.command.Robot
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

class Robot(hardwareMap: HardwareMap, gamepad1: Gamepad, gamepad2: Gamepad): Robot() {
    val hardware = Hardware(hardwareMap)
    val drive = Drive(hardware)

    val gamepads = Pair(GamepadEx(gamepad1), GamepadEx(gamepad2))

    private val limelight: Limelight3A = hardware["limelight"]
    private val odometry: GoBildaPinpointDriver = hardware["odometry"]

    init {
        register(drive)
    }

    fun start() {
        limelight.start()
        odometry.resetPosAndIMU()
    }

    fun loop() {
        limelight.updateRobotOrientation(odometry.getHeading(AngleUnit.DEGREES))
        odometry.update()

        run()
    }
}