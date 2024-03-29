# strykeInTheBot
Clone of 2910 2023 Robot

## CAN Bus

| Subsystem  | Type     | Talon                 | ID | PDP      | Motor         | Breaker |
| ---------- | -------- | --------------------- | -- | -------- | ------------- | ------- |
| Drive      | SRX      | azimuth               | 0  |          |  9015         |         |
| Drive      | SRX      | azimuth               | 1  |          |  9015         |         |
| Drive      | SRX      | azimuth               | 2  |          |  9015         |         |
| Drive      | SRX      | azimuth               | 3  |          |  9015         |         |
| Drive      | FX       | drive                 | 10 |          |  falcon       |         |
| Drive      | FX       | drive                 | 11 |          |  falcon       |         |
| Drive      | FX       | drive                 | 12 |          |  falcon       |         |
| Drive      | FX       | drive                 | 13 |          |  falcon       |         |
| Drive      | FX       | drive                 | 14 |          |  falcon       |         |
| Drive      | FX       | drive                 | 15 |          |  falcon       |         |
| Drive      | FX       | drive                 | 16 |          |  falcon       |         |
| Drive      | FX       | drive                 | 17 |          |  falcon       |         |
| Drive      | FX       | drive                 | 18 |          |  falcon       |         |
| Shoulder   | FX       | shoulderLeft1Main     | 20 |          |  falcon       |         |
| Shoudler   | FX       | shoulderLeft2Follow   | 21 |          |  falcon       |         |
| Shoulder   | FX       | shoulderRight1Follow  | 22 |          |  falcon       |         |
| Shoulder   | FX       | shoulderRight2Follow  | 23 |          |  falcon       |         |
| Extendo    | FX       | extendoMain           | 30 |          |  falcon       |         |
| Extendo    | FX       | extendoFollow         | 31 |          |  falcon       |         |
| Wrist      | FX       | wrist                 | 40 |          |  falcon       |         |
| Wrist      | Encoder  | wristEncoder          | 41 |          |  CANAndCoder  |         |
| Hand       | FX       | hand                  | 50 |          |  falcon       |         |


## Minimal Robot

### Flysky Controller (new)
![flysky](docs/Driver-Controls.png)

![operator](docs/Operator-Controls.png)

| Subsystem  | Type     | Talon                 | ID | PDP      | Motor         | Breaker |
| ---------- | -------- | --------------------- | -- | -------- | ------------- | ------- |
| Drive      | SRX      | azimuth               | 0  |   4      |  9015         |  30     |
| Drive      | SRX      | azimuth               | 1  |   11     |  9015         |  30     |
| Drive      | SRX      | azimuth               | 2  |   7      |  9015         |  30     |
| Drive      | SRX      | azimuth               | 3  |   8      |  9015         |  30     |
| Drive      | FX       | drive                 | 10 |   0      |  falcon       |  40     |
| Drive      | FX       | drive                 | 11 |   15     |  falcon       |  40     |
| Drive      | FX       | drive                 | 12 |   3      |  falcon       |  40     |
| Drive      | FX       | drive                 | 13 |   12     |  falcon       |  40     |
| Shoulder   | FX       | shoulder              | 20 |   1      |  falcon       |  40     |
| Hand       | FX       | hand                  | 50 |   13     |  falcon       |  40     |


## Roborio
| Subsystem | Interface | Device | 
| --------- | --------- | ------ |
| Drive     | USB       | NAVX   |


## DIO
| Subsystem | name       | ID |
| --------- | ---------- | -- |
| Hand      | BeamBreak  | 0  |
| Auto      | autoSwitch | 1  |
| Auto      | autoSwitch | 2  |
| Auto      | autoSwitch | 3  |
| Auto      | autoSwitch | 4  |
| Auto      | autoSwitch | 5  |
| Auto      | autoSwitch | 6  |
| Robot     | BNC        | 7  |
|           |            | 8  |
|           |            | 9  |
