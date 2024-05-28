the donwload of files can be using the API or ftp, we recoment using the ftp in the case of big files.

For know the files to download you can cosult suing the

# API donwload files

/files/download/{route\*}

# FTP

The server ftp is mounted on port 22 and only allow the access to file where save the mission result and can access using the USER and PASWORD gived for the administrator. and you can download files using the routed give for the api.

```
ftp localhost 22
```

# simulate device FTP

docker run -i --rm --mount type=bind,source=/home/grvc/work/px4,destination=/home/one/ -p 21:21 -p 21000-21010:21000-21010 -e USERS="one|1234" delfer/alpine-ftp-server
