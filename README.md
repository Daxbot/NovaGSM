# NovaGSM
Generic driver for estabilishing a TCP connection through a standard GSM/GPRS modem using AT commands
 - [repository](https://github.com/DaxBot/NovaGSM)
 - [documentation](https://daxbot.github.io/NovaGSM/index.html)

## Goals
The goals of the project are as follows:
 - be unconstrained by hardware
 - be suitable for an embedded environment (no classes or inheiritance).
 - be non-blocking
 
## Description
This driver attempts to establish communication with a GSM/GPRS modem using standard AT commands.
When the device is identified use connect() to authenticate with GPRS and open() establish a TCP stream.

Once a socket is open data can be sent or recieved using a standard read/write/available interface.
An internal ring buffer is used to receive data, and a pre-allocated pool of packets is used to queue commands to
sent to the device.  This allows the driver to be completely non-blocking without relying on complicated callbacks.

The process() state machine is below, descriptions of states can be found [here](https://daxbot.github.io/NovaGSM/namespaceGSM.html#a4d5250778227b48f8f99b4e290393fa7)
![process() state machine](https://daxbot.github.io/NovaGSM/dot_inline_dotgraph_1.png)
 
## Setup
TODO

