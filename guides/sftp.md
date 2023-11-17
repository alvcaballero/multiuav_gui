
https://www.youtube.com/watch?v=XNjOSY-wcb0&t=224s
https://www.linuxhispano.net/2011/02/07/configurar-vsftp-para-conexiones-seguras-tlssslsftp/

```
sudo apt update
sudo apt intall vsftpd


```

sudo systemctl status vsftpd
sudo nano /etc/vsftpd.conf


change:
local_enable=YES

add:
user_sub_token=$USER
local_root= /home/$USER/UAV_MEDIA


userlist_enable=YES
userlist_file=/etc/vsftpd.userlist
userlist_deny=NO

"" try to cibfigure ssl
https://www.youtube.com/watch?v=XNjOSY-wcb0&t=224s