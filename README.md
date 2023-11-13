# datc_user_interface
- The graphic user interface for KR-DATC compatible with TCP socket communication.

## TCP Socket communication test using 'telnet'
- Run 'telnet' in terminal (Window / Linux)
- If there is no 'telnet' command in Window OS, you can enable in Control Panel.
```
$ telnet localhost <socket_port>
```
- Example (command must be inserted into terminal running telnet)
```
# Json format, Datc initialize command
{"command":101}
# Json format, Gripper open command
{"command":102}
# Json format, Gripper close command
{"command":103}
```