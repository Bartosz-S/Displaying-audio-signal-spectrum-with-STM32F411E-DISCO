# Displaying audio signal's spectrum with STM32F411E-DISCO

Connections between STM32F411E-DISCO board and used peripherals - LCD display Waveshare 13892 and  DFRobot SEN0526
<img width="472" height="309" alt="image" src="https://github.com/user-attachments/assets/055ed1f4-4a2f-4a7c-8856-83937abe5c06" />

By using development board's USER button we can switch between two display states - linear (best for detecting small amplitude signals, like speaking) and dB scales (for typical audio signals and larger amplitudes).

Display in linear scale with test signal (human whistling):

<img width="717" height="439" alt="image" src="https://github.com/user-attachments/assets/7b95e936-32da-412a-a5d5-c04fa48ff33f" />


Display in dB scale with the same signal:

<img width="716" height="444" alt="image" src="https://github.com/user-attachments/assets/21cc1c82-5037-41f4-8a2d-ac44da90da52" />


Scales of two modes are shown below:

<img width="939" height="588" alt="image" src="https://github.com/user-attachments/assets/93521d76-3e92-4033-8009-fbe09cfe8c92" />
