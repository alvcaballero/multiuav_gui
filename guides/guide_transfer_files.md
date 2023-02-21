## References
* [inspector software_UAV](https://github.com/AlejandroCastillejo/inspector_software_uav)  int this proyect get the camera images from sd using ftp and save in the bag of drone   for after sent it  to the server in GCS  using  sshpass, scp and python, this run in the bag of uav.

I use that reference the last package.where i going to use  rsync and use C++.
follow this flow:

the flow it's run with console some aplication for make the steps:
 *  the gui send a service for make   the uav send the files
 *  the drone run a bash script for send the files in the folder bag
 *  when finish to send  the bag copy thay to fold backup/bags
  *  then send a response that all si sincronice

for send fieles I use this [guide](https://www.digitalocean.com/community/tutorials/how-to-use-rsync-to-sync-local-and-remote-directories-es)

    rsync -ap ~/dir1 username@remote_host:destination_directory

Now it need you put a passwork
 sudo apt-get install sshpass // nos ayuda a colocar el password

Use "sshpass" non-interactive ssh password provider utility On Ubuntu
 sudo apt-get install sshpass
 
/usr/bin/rsync -ratlz --rsh="/usr/bin/sshpass -p password ssh -o StrictHostKeyChecking=no -l username" src_path  dest_path

sshpass -p "password" rsync -ae "ssh -p remote_port_ssh" /local_dir  remote_user@remote_host:/remote_dir


sshpass -p "password" rsync -ae /local_dir  remote_user@remote_host:/remote_dir

Esta opcion sincroniza todos los archivos, tiene ip estatica.

-------------------------
Creo que conosco otra forma donde el dron publica una pagina web donde estan los archivos que se pueden descargar.
Esta me parece mas interesante ya que te permite seleccionar que archivo quieres descargar ,saber donde descargas los archivos y tener un progreso de la descarga y saber que se han descargado los archivos y no importa quien esta haciendo de GUI.

se podria hacer mediante el uso de http file server  usando apache o ubuntu de tal forma que solo se levante el servicio cuando e dron esta en tierra y apagar cuando inicie una mission
 
 y usar ln para redireccionar a la carpeta bags