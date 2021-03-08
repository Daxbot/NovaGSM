# NovaGSM
Generic driver for estabilishing a TCP connection through a SIM7000 GSM/GPRS
modem using AT commands
 - [repository](https://github.com/DaxBot/NovaGSM)

## Description
This driver attempts to establish communication with a SIM7000 GSM/GPRS modem
using standard AT commands. When the device is registered use authenticate() to
enable the data connection and connect() to establish a TCP stream.

Once a socket is open data can be sent or recieved using an asyncronous
send/receive interface.

## Setup
TODO

