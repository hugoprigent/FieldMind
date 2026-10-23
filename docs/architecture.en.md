[Français](architecture.md) · **English**

# Prototype architecture

The **Raspberry Pi 4** runs the drivers, geometric perception, pose estimation, the map, navigation and missions. The **XIAO ESP32-C5** reads the sensors and applies the traction and servo setpoints; its watchdog cancels traction if the setpoints stop. The web station displays the sensors, the map and the mission, and allows manual driving.

The AI agent proposes structured tool calls. The software checks their arguments and turns them into mission actions; the commands go through the same supervisor as the other driving modes. The model does not write the motor PWM directly.

The `gx10` profile uses APIs of models running on the **GB10 of the ASUS Ascent GX10**. The `aucun` profile allows simulation and station commands without any AI service. An external Gemini profile remains available for comparison and development; it must be selected explicitly and requires a key.

The current simulation replaces the hardware drivers: a fake ESP using the same protocol, and simulated lidar, stereo images and GNSS. The rest of the chain is the rover's. The first MuJoCo demo in `software/services`, `software/simulation` and `software/interface` is a separate stack, kept to reproduce the online demo.

The drone and the trailer are experimental extensions. The drone scenarios are simulated; a video protocol or a simulation report does not prove a real autonomous flight.

For the detailed interfaces: [rover architecture](../software/rover/ARCHITECTURE.en.md), [web protocol](../software/rover/PROTOCOLE.en.md), [firmware](../software/esp32/esp_c5/README.en.md), [AI deployment](../software/rover/deploiement/gx10/README.en.md).
