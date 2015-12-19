## industrial-switcher

This board is meant for switching various 24V loads, like coils and relays.

## Requirements

[PlatformIO](http://platformio.org) is used for building.

```
pip install -U pip setuptools
pip install -U platformio
platformio platforms install timsp430
```

## Build and upload
```
platformio run
platformio run --target upload
```

## License
