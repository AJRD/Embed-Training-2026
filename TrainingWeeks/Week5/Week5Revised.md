# Week 5 Revised
Okay, so now you've basically done with all of the trainings, except for two things: DeviceTree and Ozone.

So far in the trainings, you've been working with a mock up of mbedOS (no relation to TR-embed lol, I was fairly confused when I first heard about it), which is our old RTOS. Since this training was written, we have ported to a new RTOS, zephyrOS, but first, what even is an RTOS?

## What is an RTOS? 

RTOS is an acronym that stands for "Real-Time Operation System". Starting with latter half, an operating system is just software that manages the operations (processes, memory, etc.), and some examples include Windows, MacOS, and Linux. Any OS will give you a scheduler, which is basically a set of tools that let you decide when something should execute; however, the power of an RTOS is that our scheduler is deterministic. In Windows for instance, the scheduler optimizes performance, and so you can have variance in the duration of tasks and when they're called. This is fine in desktop application since users generally don't care about variance on the order of milliseconds; however, our controls loop and peripherals definitely do. 

This is where the real-time aspect comes in. An RTOS scheduler differs from a standard OS scheduler in that it's built to ensure deterministic loops, so time-critical tasks are always executed on time and not early or late. You've seen some aspects of this in the training so far, primarily with `Thisthread::sleep_for`, but it goes much farther than that. If you're interested, contact me and we can talk more about scheduling, but honestly you can go your entire time at TR without having to really care that much about it.

### Beyond Scheduling: DeviceTree

Apart from scheduling, RTOS's also give us dedicated tools (application programming interface, or APIs for short) to manage a lot of the lower-level software problems we face. In almost all RTOSes, you'll find a similar set of low-level APIs dedicated to common communication protocols: I2C, SPI, CAN, etc. and amongst these APIs, every RTOS has to decide how they want to manage your I/O. In mbedOS, we did it rather explicitly, by just directly setting pins at runtime logic. However, a consequence of this is that it can be rather messy. For example, let's say that we want to change our wiring since electronics gave us a new PCB, well then we will have to manually find each instance where a pin is called and reassign it; or alternatively, let's just say that we want to reconfigure i2c from running at 100khz to 400khz, since we realized that we can run it on fast mode. In any case, it can be somewhat confusing to find exactly where we configure our I/O and often times seemingly related changes, like changing the IMU pin or the IMU hz can be in entirely different files. That's fine if you've directly worked with the peripheral before, but if you haven't, it can be a headache to dig through the files.

This leads us to the alternative to explicitly configuring our I/O: abstracting it. Namely, we do this in ZephyrOS with the devicetree. 

[TODO: INCLUDE IMAGE HERE]

What devicetree is at its core is a list of structs that dictate the configuration for our I/O, and each one of these structs is called a node. These nodes primarily consist of communication protocol busses (i2c, can, uart, etc.). In our case, since our microcontroller is supported directly by zephyr, it already has a internal `.dtsi` file that has the information for each node, and so we only need to enable/disable them, which ends up being a fairly simple process, since we just write a .overlay file `nucleo_f446re.overlay` to be exact, and that sets the configuration we want. For example, let's examine our can1 node.

```C++
&can1 { //The node is labeled with &can1, which just lets zephyr know to reference its internal can1 setting for our board
    pinctrl-0 = <&can1_rx_pa11 &can1_tx_pa12>; // pinctrl-0 just sets the pins we want, in this case we're using & again since these are specific pins within the .dtsi
    pinctrl-names = "default"; // This just lets the compiler know we're using the default names and not some alias
    status = "okay"; // status is what enables/disables the node; okay means enabled, "disabled" means disabled
    bitrate = <1000000>; // This is just the can baud rate, different communication protocols have diff keywords for this same idea 
};
```

