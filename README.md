### Clone repository
```
git clone --recursive git@github.com:zynquinopi/uwbins.git
```

### Attach USB Device to WSL2
```
@Windows
usbipd bind --force --busid 2-2
ssh {user}@localhost -R 3240:localhost:3240

@WSL2
sudo usbip attach -b 2-2 -r 127.0.0.1
```

### Update spresense sdk (Execute only once at the beginning)
```
cd external/spresense
source spresense_env.sh
sdk/tools/flash.sh -e ../../etc/spresense-binaries-v3.4.1.zip
sdk/tools/flash.sh -l firmware/spresense -c /dev/ttyUSB0
```

### Register our application in the spresense sdk (Execute only once at the beginning)
```
mkdir -p external/spresense/examples/uwbins
sudo mount --bind spresense_app external/spresense/examples/uwbins
```

### Build, upload and execute
```
cd external/spresense
sudo update-alternatives --install /usr/local/bin/usbip usbip /usr/lib/linux-tools/*/usbip 20
sudo mount --bind {path/to/uwbins/absolute/path}/spresense_app examples/uwbins
spresense tools/config.py -d ../examples/uwbins/configs release
    (feature/wifi feature/libcxx examples/udp)
spresense make -j
sdk/tools/flash.sh -b 115200 -c /dev/ttyUSB0 sdk/nuttx.spk
minicom -D  /dev/ttyUSB0 115200
```

### Upload startup script
```
sdk/tools/flash.sh -w examples/uwbins/init.rc
```

### Update defconfig
```
cd external/spresense
spresense tools/mkdefconfig.py -d ../examples/uwbins release
```
