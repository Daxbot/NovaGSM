# NovaGSM
Generic driver for estabilishing a TCP connection through a standard GSM/GPRS modem using AT commands
 - [repository](https://github.com/DaxBot/NovaGSM)
 - [documentation](https://daxbot.github.io/NovaGSM/index.html)

## Goals
The goals of the project are as follows:
 - be unconstrained by hardware
 - be suitable for an embedded environment (no virtual classes or dynamic memory allocation).
 - be non-blocking
 
## Description
This driver attempts to establish communication with a GSM/GPRS modem using standard AT commands.
When the device is identified use authenticate() to register with GPRS and connect() to establish a TCP stream.

Once a socket is open data can be sent or recieved using an asyncronous send/receive interface.
A pre-allocated pool of memory is used to queue device commands.
Reserved memory can be tuned with -DGSM_BUFFER_SIZE and -DGSM_POOL_SIZE for resource constrained environments. 

![process() state machine](https://daxbot.github.io/NovaGSM/dot_inline_dotgraph_1.png)
 
## Setup
TODO

