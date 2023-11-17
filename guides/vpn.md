Seting of vpn

There is no "/etc/rc.local" file on Ubuntu 18.04, but you can create it.

Create the file with a text editor:

sudo nano /etc/rc.local
Paste the following lines and replace "COMMANDS" with the commands to be executed at system startup:

```
#!/bin/sh -e
COMMANDS
exit 0
Add the execute permission on the file:
```

```
chmod +x /etc/rc.local
```
