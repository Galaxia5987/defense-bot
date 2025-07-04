package frc.robot.subsystems.leds

import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase

class LEDs : SubsystemBase() {
    private val ledStrip =
        AddressableLED(LED_STRIP_PORT).apply { setLength(STRIP_LENGTH) }
    private val ledBuffer = AddressableLEDBuffer(STRIP_LENGTH)
    private val leftBuffer = ledBuffer.createView(0, STRIP_LENGTH / 2)
    private val rightBuffer =
        ledBuffer.createView(STRIP_LENGTH / 2, STRIP_LENGTH - 1)

    init {
        ledStrip.start()
        setPattern(climbPattern)
    }

    fun setPattern(
        left: LEDPattern? = null,
        right: LEDPattern? = null,
        all: LEDPattern? = null
    ): Command =
        run {
                left?.applyTo(leftBuffer)
                right?.applyTo(rightBuffer)
                all?.applyTo(ledBuffer)
            }
            .ignoringDisable(true)

    fun setPatternArea(
        primaryPattern: LEDPattern,
        secondaryPattern: LEDPattern? = null,
        section: Array<Int>
    ): Command =
        run {
                val sectionOfBuffer: AddressableLEDBufferView =
                    ledBuffer.createView(section[0], section[1])
                secondaryPattern?.applyTo(ledBuffer)
                primaryPattern.applyTo(sectionOfBuffer)
                ledStrip.setData(ledBuffer)
            }
            .ignoringDisable(true)

    override fun periodic() {
        ledStrip.setData(ledBuffer)
    }
}
