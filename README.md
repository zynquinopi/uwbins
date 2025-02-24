### Update spresense sdk (Execute only once at the beginning)
```
cd external/spresense
source spresense_env.sh
sdk/tools/flash.sh -e ../../../etc/spresense-binaries-v3.3.0.zip
sdk/tools/flash.sh -l firmware/spresense -c /dev/ttyUSB0
```

### Register our application in the spresense sdk (Execute only once at the beginning)
```
mkdir -p external/spresense/examples/uwbins
sudo mount --bind spresense_app external/spresense/examples/uwbins
```

### 