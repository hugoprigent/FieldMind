[Français](README.md) · **English**

<a id="logiciels-esp32-du-rover"></a>
# ESP32 Rover Software

XIAO ESP32-C5 Project: [esp_c5/](esp_c5/README.en.md). Open software/esp32/esp_c5 in VS Code with PlatformIO.

The default program **esp_c5 / ROVER_2** integrates both motors, the D7 servo, the screen, the IMU, the barometer, and the optional Knob. See [the Raspberry guide and pinout](esp_c5/CONNEXION_RASPBERRY.en.md). Pi supervision remains in embarque/ (non-included working reference).

test_tout_moteur reuses the same controller, without sensors. Standalone motor tests (D0) and standalone servo tests (D0, disconnected from the driver) remain available in their separate environments.

USB 115200: m 10, m 0, s -10, center, v 30, stop, status. The main publishes sensors in the format already decoded by the Pi. USB reading/outputs are separate from I2C. The proposed JSON contract and autonomous navigation are not implemented by this change.

To verify logic on PC: python tests/verifier_policy.py.
