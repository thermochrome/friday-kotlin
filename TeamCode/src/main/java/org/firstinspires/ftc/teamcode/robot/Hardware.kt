package org.firstinspires.ftc.teamcode.robot

import com.arcrobotics.ftclib.hardware.motors.CRServo
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap

class Hardware(private val hardwareMap: HardwareMap) {
    object Native

    val objects: MutableMap<String, Any> = LinkedHashMap()

    private val objectData = mapOf(
        "left_front" to MotorEx::class,
        "right_front" to MotorEx::class,
        "left_back" to MotorEx::class,
        "right_back" to MotorEx::class,
        "outtake_l" to MotorEx::class,
        "outtake_r" to MotorEx::class,
        "intake" to MotorEx::class,

        "feeder" to Native,
        "odometry" to Native,
        "limelight" to Native
    )

    init {
        val devices = objectData.map { (name, type) ->
            val device = when (type) {
                MotorEx::class -> MotorEx(hardwareMap, name).apply {
                    setRunMode(Motor.RunMode.RawPower)
                }

                CRServo::class -> CRServo(hardwareMap, name)

                Native -> {
                    val device = hardwareMap.get(type.javaClass, name)

                    when (name) {
                        "odometry" -> (device as GoBildaPinpointDriver).setEncoderResolution(
                            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)

                        "limelight" -> (device as Limelight3A).pipelineSwitch(0)

                        else -> device
                    }
                }

                else -> throw IllegalArgumentException("Could not find $name")
            }

            name to device
        }.toMap()

        objects.putAll(devices)
    }

    inline operator fun <reified T> get(name: String): T {
        val device = objects[name] ?: throw IllegalArgumentException("$name not found")

        if (device !is T) {
            throw IllegalArgumentException("$name is not a ${T::class.simpleName}")
        }

        return device
    }
}