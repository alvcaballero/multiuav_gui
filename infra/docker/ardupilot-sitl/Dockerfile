# ArduPilot SITL (sin simulador gráfico) para probar el GCS por MAVLink.
# Reconstruido de las capas de `muav-arducopter-sitl:4.7.2` (su Dockerfile original se perdió en un directorio
# temporal). ArduPilot es UN repo con un tag por vehículo; el vehículo se elige al compilar.
#
#   docker build -t muav-arducopter-sitl:4.7.2 infra/docker/ardupilot-sitl
#   docker build -t muav-ardurover-sitl:4.7.2 \
#     --build-arg AP_TAG=Rover-4.7.2 --build-arg AP_VEHICLE=rover infra/docker/ardupilot-sitl
#
#   docker run --rm --network=host -e GCS_OUT=udpclient:127.0.0.1:14550 muav-arducopter-sitl:4.7.2
#
# AP_VEHICLE: copter | rover (el binario es build/sitl/bin/ardu<vehículo>, los params
# Tools/autotest/default_params/<vehículo>.parm).
FROM ardupilot/ardupilot-dev-base:v0.0.29

ARG AP_TAG=Copter-4.7.2
ARG AP_VEHICLE=copter
WORKDIR /ardupilot

RUN git clone --depth 1 --branch ${AP_TAG} --recurse-submodules --shallow-submodules \
      https://github.com/ArduPilot/ardupilot.git . \
 && ./waf configure --board sitl \
 && ./waf ${AP_VEHICLE}

# El System ID de la unidad en el GCS (`attributes.systemId`); pisarlo con -e SYSID=<n>.
ENV VEHICLE=${AP_VEHICLE} \
    SYSID=2 \
    HOME_LOC=43.4726046,-8.1788475,100,0 \
    GCS_OUT=udpclient:127.0.0.1:14560

# copter vuela el modelo "+", rover el "rover": el nombre del modelo es el del vehículo salvo en copter.
ENTRYPOINT ["sh", "-c", "printf 'MAV_SYSID %s\\n' \"$SYSID\" > /ardupilot/gcs.parm && exec build/sitl/bin/ardu${VEHICLE} --model $([ \"$VEHICLE\" = copter ] && echo + || echo $VEHICLE) --speedup 1 -I0 --home ${HOME_LOC} --defaults Tools/autotest/default_params/${VEHICLE}.parm,/ardupilot/gcs.parm --serial0 ${GCS_OUT}"]
