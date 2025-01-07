
:: Habilitar acceso al servidor X para Docker en Windows
setlocal enabledelayedexpansion

:: Configurar las rutas del proyecto
set PROJECT_DIR=%USERPROFILE%\work\px4
set PROJECT_DIST=/home/nonroot/work


:: Reemplazar la barra invertida "\" por "/"
set PROJECT_DIR_UNIX=!PROJECT_DIR:\=/!

:: Convertir el prefijo de unidad ("C:") a "/c"
set DRIVE_LETTER=!PROJECT_DIR_UNIX:~0,1!
set PROJECT_DIR_UNIX=/%DRIVE_LETTER%!PROJECT_DIR_UNIX:~2!

:: Imprimir la ruta convertida
echo Ruta en formato UNIX: %PROJECT_DIR_UNIX%
:: Mostrar la ruta actual del proyecto
echo Current working directory: %PROJECT_DIR%

:: docker run -it --name px4_noetic3 --privileged --env DISPLAY=:0 --network host --pid=host --mount type=bind,source=/c/Users/arpag/work,destination=/home/nonroot/work --volume /run/desktop/mnt/host/wslg/.X11-unix:/tmp/.X11-unix --volume /run/desktop/mnt/host/wslg:/mnt/wslg muavgcs:noetic bash 

:: Verificar si el contenedor ya existe
set CONTAINER_ID=
for /f "tokens=*" %%i in ('docker ps -q -a --filter "name=px4_noetic"') do (
    set CONTAINER_ID=%%i
)
echo CONTAINER_ID: [%CONTAINER_ID%]
if "%CONTAINER_ID%"=="" (
    echo Container not found, creating it ...
    docker run -it ^
        --name px4_noetic ^
        --privileged ^
        --workdir %PROJECT_DIST% ^
        --env DISPLAY=%DISPLAY% ^
        --network host ^
        --pid=host ^
        --volume /run/desktop/mnt/host/wslg/.X11-unix:/tmp/.X11-unix  ^
        --volume /run/desktop/mnt/host/wslg:/mnt/wslg ^
        --mount type=bind,source=%PROJECT_DIR_UNIX%,destination=%PROJECT_DIST% ^
        muavgcs:noetic bash
) else (
    echo Container already exists: %CONTAINER_ID%
    if "%1"=="restart" (
        echo Restarting container...
        docker stop px4_noetic >nul || (
            echo Error while stopping the container, exiting now...
            exit /b 1
        )
    )
    docker start px4_noetic >nul || (
        echo Error while starting the container, exiting now...
        exit /b 1
    )
    echo px4_noetic found and running, executing a shell...
    docker exec -it px4_noetic bash --login
)
