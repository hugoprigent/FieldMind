[Français](INTERFACE_USB.md) · **English**

<a id="pilotage-depuis-le-navigateur-par-usb"></a>
# Control from the browser, via USB

Launch in this folder:

```powershell
python lancer_interface_usb.py
```

The page is served only on this PC, at http://127.0.0.1:8767/. Open this address in **Chrome or Edge**. No Wi-Fi, account, external service, or Python module installation is required. The server only reads the `interface_usb/` folder and does not open the serial port itself.

1. The ESP must have the **esp_c5 / ROVER_3** (or ROVER_2) or **test_tout_moteur / TOUT_MOTEUR_4** firmware.
2. Close the PlatformIO serial monitor, then click **Connect ESP**.
3. Select the ESP32-C5 port in the browser selector.
4. Set a value and click **Apply Speed** or **Apply Angle**.

The page only sends `etat` to the connection. The firmware already centers the servo at startup; traction remains at zero without an explicit command. Both drivers receive the same percentage. Servo on D7/GPIO12, traction on D0/D9, D/R separated on D1/D4; see [the combined test wiring](TEST_TOUT_MOTEUR.en.md).

The current firmware centers the servo at startup (1500 us). To set the zero, disconnect the linkage, then click **Center at 0**: setpoint of 1500 us. The first movement may be fast, as the physical position is unknown. Then cut the servo power to reposition the horn without turning the shaft, direction straight ahead.

The **Motors at zero** button keeps the direction signal. **STOP** or **Esc** sends `stop`: zero traction PWM and servo signal cut. The STOP button also removes commands that are still waiting to be sent in the page. The displayed positions are setpoints, not actual position measurements. The Left and Right blocks adjust each percentage independently. Forward/Backward only changes the direction of the selected wheel, wheel at 0 % and physical stop required. Correcting the forward reverses the reference setting of this wheel and memorizes it; both motors must be at zero. ROVER_1 and TOUT_MOTEUR_3 are rejected by this updated page: upload ROVER_3 or TOUT_MOTEUR_4 for the new controls. ROVER_3 is accepted because it keeps ROVER_2 commands; the page does not use `p`, `wd`, `tel` nor `imu`. If the Pi left `tel` or `imu` active, send `tel 0` and `imu 0` (or reconnect USB) to restore the periodic state; `wd 0` if the watchdog remained armed.

The browser reads serial messages and blocks movement controls if the firmware is not recognized or if the ESP reports a fault. The serial return remains visible, with the fault pattern. A refused connection often indicates that VS Code or another software is still using the port.

STOP is requested before disconnection and when the tab becomes hidden. Abrupt browser closure, PC failure, or a blocked link do not guarantee delivery of this message: use STOP before leaving the page. This does not cut the power supplies and does not replace a physical rover shutdown. There is no automatic movement command after reconnection and the page does not modify the firmware.

To close the server launched in a terminal: Ctrl+C. If the server is already running, relaunching the script reopens the existing page. `--no-browser` launches it without opening a window; `--port 8768` allows changing its HTTP port.

Implementation: [Web Serial API documented by Chrome](https://developer.chrome.com/docs/capabilities/serial). All HTML/CSS/JavaScript is local; no remote resources are loaded.

Verification of 18 October 2026: 4 transport tests with `node --test tests/interface_usb.test.mjs`, Chrome browser test with simulated serial port (connection without movement, m/s/v commands, STOP, fault, incompatible firmware, mobile display). No access to a real USB port or physical movement during verifications. The `tests/interface_usb_browser_test.mjs` script uses Playwright installed in the npm cache and the PLAYWRIGHT_MODULE variable pointing to its index.mjs file; it never requests a real port.
