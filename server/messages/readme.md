complile :

```
./flatbuffers/flatc --ts --gen-all --gen-onefile -o ./messages  ./schemas/schema_main.fbs
./flatbuffers/flatc --cpp --gen-all --gen-onefile -o ./messages2  ./messages/schema_main.fbs
./flatbuffers/flatc --cpp --gen-all -n ./messages2  ./messages/schema_main.fbs

```
