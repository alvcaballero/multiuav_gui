https://download.geofabrik.de/europe/spain.html

https://switch2osm.org/serving-tiles/using-a-docker-container/

```
docker run  -v /home/user/work/map/spain-latest.osm.pbf:/data/region.osm.pbf  -v osm-data:/data/database/  overv/openstreetmap-tile-server  import

```

For create the container with no cors

```
    docker run -p 8080:80 --name offline_map -v osm-data-andalucia:/data/database -e ALLOW_CORS=1 -d overv/openstreetmap-tile-server  run
```

For run the container

```
docker start 0f77a2f7ea1d
```

#Problem with Glyphs
https://github.com/openmaptiles/fonts#open-font-glyphs-for-gl-styles
https://github.com/Geodan/glyphserver

for import or export volume of maps i use docker volume snapshot

https://github.com/junedkhatri31/docker-volume-snapshot
