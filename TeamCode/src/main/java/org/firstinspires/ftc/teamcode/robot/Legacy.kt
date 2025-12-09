package org.firstinspires.ftc.teamcode.robot

//class Hardware(private val hardwareMap: HardwareMap) {
//    object Native
//
//    val objects: MutableMap<String, Any> = LinkedHashMap()
//
//    private val objectData = mapOf(
//        "left_front" to MotorEx::class,
//        "right_front" to MotorEx::class,
//        "left_back" to MotorEx::class,
//        "right_back" to MotorEx::class,
//        "outtake_l" to MotorEx::class,
//        "outtake_r" to MotorEx::class,
//        "intake" to MotorEx::class,
//        "upper_intake" to CRServo::class,
//
//        "feeder" to Native,
//        "odometry" to Native,
//        "limelight" to Native
//    )
//
//    init {
//        val devices = objectData.map { (name, type) ->
//            val device = when (type) {
//                MotorEx::class -> MotorEx(hardwareMap, name).apply {
//                    setRunMode(Motor.RunMode.RawPower)
//                }
//
//                CRServo::class -> CRServo(hardwareMap, name)
//
//                Native -> {
//                    when (name) {
//                        "odometry" -> {
//                            val device = hardwareMap.get(GoBildaPinpointDriver::class.java, name)
//
//                            device.setEncoderResolution(
//                                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
//                            device
//                        }
//
//                        "limelight" -> {
//                            val device = hardwareMap.get(Limelight3A::class.java, name)
//
//                            device.pipelineSwitch(0)
//                            device
//                        }
//
//                        "feeder" -> {
//                            val device = hardwareMap.get(Servo::class.java, name)
//                            device
//                        }
//
//                        else -> throw IllegalArgumentException("Could not find $name")
//                    }
//                }
//
//                else -> throw IllegalArgumentException("Could not find $name")
//            }
//
//            name to device
//        }.toMap()
//
//        objects.putAll(devices)
//    }
//
//    inline operator fun <reified T> get(name: String): T {
//        val device = objects[name] ?: throw IllegalArgumentException("$name not found")
//
//        return device as T
//    }
//}