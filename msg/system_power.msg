uint64 timestamp        # time since system start (microseconds)

float32 voltage5v_v     # peripheral 5V rail voltage
float32 voltage3v3_v    # Sensor 3V3 rail voltage
bool v3v3_valid         # Sensor 3V3 rail voltage was read.

bool usb_connected      # USB is connected when 1
bool usb_valid          # USB is valid when 1

bool servo_valid        # servo power is good when 1

bool periph_5v_oc       # peripheral overcurrent when 1
bool hipower_5v_oc      # hi power peripheral overcurrent when 1


uint8 BRICK1_VALID_SHIFTS=0
uint8 BRICK1_VALID_MASK=1
uint8 BRICK2_VALID_SHIFTS=1
uint8 BRICK2_VALID_MASK=2
uint8 BRICK3_VALID_SHIFTS=2
uint8 BRICK3_VALID_MASK=4
uint8 BRICK4_VALID_SHIFTS=3
uint8 BRICK4_VALID_MASK=8

bool brick_valid         # brick bits power is good when bit 1
