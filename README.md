# Compilation of Projects and assignments for CPE439 Real-Time Operating Systems<br /> #


<details>
  <summary><strong>Projects</strong></summary>

  ### P1: Spectrum Analyzer
  
  > This project designs a spectrum analyzer on the STM32 Microcontroller utilizing FreeRTOS to manage different tasks deterministically, and continuously process new data in real-time. The spectrum analyzer is capable of measuring and graphing any kind of signal with a Vpp between 0-3.3V and a DC offset between 0-3.3V. It includes a frequency range between 1 Hz to 1 kHz, graphed with 64 frequency bins at a refresh rate of 1 second.

  ### P2: Wireless Chat
  
  > This project utilizes FreeRTOS on the STM32 Microcontroller with the X-NUCLEO-IDS01A5 expansion board to construct a chat program. The program implements a network protocol such that each node can unicast messages via user-inputted usernames, broadcast to all active users, and print out the usernames of all the active nodes. All users are given a unique 8-bit address, which gets mapped to the usernames of each node. A “heartbeat” is broadcast every 30 seconds to indicate that the node is still online. If a node has not been heard from in 110 seconds, the program will assume it is offline. Packets are in the format of the Spirit1 STack Packet

  ### P3: Personal Design Project - Skittle Sorter

  > The system uses the TCS3472 color sensor to sort Skittles based on their colors and place them into corresponding bins. It’s capable of sorting red, green, purple, orange, and yellow, by sampling values of red, green, blue, and what they call “clear.” The data given by the Color Sensor is processed by software, in order to identify the remaining colors of purple, orange, and yellow. Motor functions are done via two HS-422 servos capable of moving .21sec/60°. One servo transports a skittle, while the other servo places the skittle into the correct bin. Due to current limitations (each servo operating at 150 mA with a max output from the STM32 5V pin rated at 150 mA) only one servo is able to operate at a time. Utilizing FreeRTOS, different tasks are designed for grabbing the skittle, scanning for the skittle’s color, and moving the skittle to the appropriate bin. A funnel allows the user to “buffer” their skittles so they don’t have to place each skittle one by one continuously. 
</details>

<details>
  <summary><strong>Assignments</strong></summary>

  ### A1: Controlled Blinky
  
  > Blinks the onboard LED using a timer. The frequency of the blinking is determined by a user input via a serial terminal interfaced using the UART protocol. The input prompt asks the user to enter the number of milliseconds the LED will be on for.

  ### A2: FreeRTOS Tasks
  
  > Introduction to FreeRTOS and Tasks. Creates 4 tasks to toggle a GPIO pin. Each task toggles a seperate pin at specified intervals: 2, 3, 5, and 15ms.

  ### A3: DMA
  > * Part A: Measuring Copying Data Speed DMA vs CPU\
  Measures how long it takes to copy a filled uint16_t array into an empty array in software (CPU) using a for loop, then how long it takes using DMA. Measurements are made by toggling GPIO pin(s) and capturing the time on an oscilloscope.
      
  > * Part B: DMA with ADC\
  Setup an ADC at 3.3V ref to take continuous 12-bit samples at a rate of 2048 Hz, triggered by a Hardware Timer I set up. Samples are moved into an array by using a configured DMA channel. 
  Interrupts are set at half-full and full instances of the array to measure and verify timing of the ADC sampling rate.

  ### A4: ARM FFT Library
  > Uses "A3 Part B" to sample ADC values into an array using DMA. An FFT is computed on these samples, utilizing the CMSIS DSP library. The frequency of an input waveform is then printed to a serial terminal via UART.

  ### A5: RTOS Watchdog
  > Introduction to Watchdog Timers with FreeRTOS. Two tasks are created.
  > * An Init Task that quickly blinks the onboard LED 5 times and initializes the watchdog timer with a timeout of 5 seconds.
  > * A Counting Task that toggles a GPIO pin (PC0) every 10 ms. This is probed via an oscilloscope.\
  A button on the Nucleo Board (PC13) is configured to cause an interrupt when pressed. The ISR resets the watchdog timer's count so the system is not reset.
  If the button is not pressed, the system will restart and the LED will blink 5 times. Otherwise, if the button is pressed, the watchdog timer is reset and PC0 will continue to toggle.

  ### A6: Hello HAL
  > Simple assignment to measure the overhead of Hardware Abstraction Layer and Low-Layer as compared to register manipulation.
  > In each instance, a GPIO pin is toggled as quickly as possible, and its period is measured to see timing differences.

  ### A7: Wireless Ping
  > Configured the SPIRIT1 module to wirelessly communicate with other students in the class. A packet is as shown:\
  > | Type | Username | Payload |\
  > These packets are built into the payload of the STack packet-type by the SPIRIT1 Library.\
  > A procedure for starting up and initially connecting to all other available nodes, recognizing when new nodes join the network, and removing nodes that have dropped off the network. 
    To verify the active status of our device in the network, it will print out all currently active nodes on the network to a terminal (USART). This listing will include the node’s address and the     time since they sent out an active (alive) ping. A "heartbeat" is sent out periodically to advertise that we are online.

  ### A8: Power Modes
  > Simple assignment to play with low-power modes in FreeRTOS with the STM32. The system runs in tickless idle and goes to sleep if idle for 50ms
  > Two FreeRTOS tasks are created:
  > * One task that waits for a Semaphore, then blinks an LED 5 times
  > * Another task that gives a Semaphore then waits for 5 seconds.\
  > An interrupt for the push button that also gives a Semaphore to trigger Task 1 above.
  > This will cause the system to go to sleep if no Semaphore is given to Task 1. Power is calculated by supplying a known voltage to the microcontroller, and measuring the IDD in both tickless idle and sleep modes.

</details>

