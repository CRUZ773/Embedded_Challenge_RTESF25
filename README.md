# Embedded_Challenge_RTESF25

Objective:
● Use the data collected from a single accelerometer and gyroscope on your dev board to detect tremors or dyskinetic hand movements, as well as a “freezing gait” condition, as exists in patients with Parkinson’s disease. Use the integrated BLE radio to transmit information on all three conditions to your mobile device on three different BLE characteristics on a single service.
Motivation:
● One of the primary symptoms of Parkinson’s disease is tremors in a dominant arm. Tremors are defined as rhythmic oscillations in the frequency range of 3- 5Hz. The treatment for these tremors is typically a dopamine boost, which transitions the patient from the “Off” state to the “On” state. However, if too much dopamine is provided, an additional symptom called dyskinesia occurs. This is characterized by rhythmic dance-like movements in the frequency range of 5-7Hz. It is essential to have this information to properly medicate a patient so that the patient is “On”, but not too “On”. In addition, late-stage patients also experience “freezing gait” episodes. This is characterized by a sudden freezing of the body, after a period of walking (detected using step detection, cadence etc.) These episodes often result in an emergency room visit.

Characteristics:
● Our development board integrates an accelerometer/gyro capable of detecting
acceleration and/or rotation in three directions. It is capable of acquiring data at
the required rate (52Hz) and the required resolution (+-2G) to detect both
movement conditions and the required measurements for “gait freezing”.
● Our system captures 3-second intervals of data that you can process using an FFT library to return information on the frequency distribution of the data in the 3-second interval. We included the FFT library to properly read the frequency. In general, all will work the same way. We provided an array of data and a sampling frequency. It then provides the frequency content of the data. 
● You should only use the development board resources to creatively control and
indicate if either of these conditions exists, as well as the intensity levels. This
includes buttons, LEDs, a BLE radio, etc…
● You can power our device with a simple power bank.
● Only our controller dev board can be used. No additional hardware is needed.
