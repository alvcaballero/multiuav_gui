
Confugurar el wifi  ingresar usuario y contraseña
    
    nmtui

ver prioridad de internet

    ip r

si primero esta ethernet borrarlo

    sudo ip r delete default via 10.42.0.1 dev eth0 proto static

cambiar la fecha y la hora para que pueda hacer el update 

    sudo date --set "2023-06-23 15:03"

