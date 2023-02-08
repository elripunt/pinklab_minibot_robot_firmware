## Template

| Header |      | CMD  | DATA ... | Response | Length | Checksum | Footer |       |
| ---    | ---  | ---  | ---      | ---      | ---    | ---      | ---    | ---   |
| 0xFA   | 0xFE | 0x01 | 0x00 ... | 0x01     | 0x0    | 0x0      | 0xFA   |  0xFD |


### Description
#### CMD
- 0x01 : Enable Motors
- 0x02 : Set TARGET speeds for both motors
- 0x03 : Request states
- 90+CMD : Response from arduino

#### DATA
- N bytes


#### Response
- 0x01 : Send response for request frames
- 0x00 : Do not send response

#### Length
- CMD + DATA + RESPONSE

#### Checksum
- CMD + DATA + RESPONSE + LENGTH
- Casting uint8_t


## Examples

### Enable Motors

PC -> Arduino

| Header |      | CMD  | DATA | Response | Length | Checksum | Footer |       |
| ---    | ---  | ---  | ---  | ---      | ---    | ---      | ---    | ---   |
| 0xFA   | 0xFE | 0x01 | 0x01 | 0x01     | 0x3    | 0x6      | 0xFA   |  0xFD |

Arduino -> PC

| Header |      | CMD  | DATA | Response | Length | Checksum | Footer |       |
| ---    | ---  | ---  | ---  | ---      | ---    | ---      | ---    | ---   |
| 0xFA   | 0xFE | 0x91 | 0x01 | 0x01     | 0x3    | 0x96      | 0xFA   |  0xFD |

### Set command for Motors

PC -> Arduino

| Header |      | CMD  | DATA |       |       |       |       |        |         |  Response | Length | Checksum | Footer |       |
| ---    | ---  | ---  | ---  |  ---  | ---   | ---   | ---   | ---    |  ---    |  ---      | ---    |  ---     | ---    | ---   |
| 0xFA   | 0xFE | 0x02 | 0x01 |  0x00 | 0x00  | 0x00  | 0x00  | 0x0    |  0x0    |  0x0      |   0x9  | 0xC      | 0xFA   |  0xFD |
|        |      |      | EN   |  LEFT |       | RIGHT |       | L_LAMP |  R_LAMP |           |        |          |        |       |


Data:
- Left Motor : 2bytes (int16_t) (High 8bytes | Low 8bytes)
- Right Motor : 2bytes (int16_t) (High 8bytes | Low 8bytes)


### Request States

PC -> Arduino

| Header |      | CMD  | Length | Checksum | Footer |       |
| ---    | ---  | ---  | ---    | ---      | ---    | ---   |
| 0xFA   | 0xFE | 0x03 | 0x1    | 0x4      | 0xFA   |  0xFD |

Arduino -> PC

| Header |      | CMD  | DATA |       |     |     |     |         |     |     |     |        |        |        |        |  Length | Checksum | Footer |       |
| ---    | ---  | ---  | ---  | ---   | --- | --- | --- | ---     | --- | --- | --- | ---    | ---    |   ---  | ---    |   ---   | ---      | ---    | ---   |
| 0xFA   | 0xFE | 0x93 | 0x01 | 0x0   | 0x0 | 0x0 | 0x0 | 0x0     | 0x0 | 0x0 | 0x0 | 0x0    | 0x0    | 0x0    | 0x0    |  0x0E   | 0xA1     | 0xFA   | 0xFD  |
|        |      |      | EN   | L_POS |    |      |     | R_ROS   |     |     |     | L_LAMP | R_LAMP | Sonar Sensor  |        |         |          |        |       |
