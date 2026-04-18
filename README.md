# simple-elm

A small Python library for working with ELM327 OBD/CAN adapters over serial.

## Connect to ELM327 device
The device I used was a ELM327 device by Veepeak, which can be sourced from [amazon](https://a.co/d/0aZCQ6Mv). You will need to connect this to your OBD port, typically under the steering wheel of your car. Open up your laptop and connect to the ELM327 device over bluetooth.

## Install locally

```bash
pip install -e .
```

## Usage
Select the COM port (in windows) the baudrate is typically set at 38400
```python
from simple_elm import ELM327, OBDPid

with ELM327(port="COM9", baudrate=38400, timeout=1.0) as elm:
    elm.initialize(protocol=6, headers=True)
    rpm_response = elm.request_pid(OBDPid.ENGINE_RPM, header="7DF")
    print(rpm_response)
```

## Main functions

The library is built around 3 primary functions:

1. `request_pid(...)` - send OBD mode+PID requests and read ECU responses.
2. `send_can(...)` / `send_data(...)` - send raw CAN/diagnostic payloads.
3. `start_monitor_can_id(...)` - monitor and stream frames for a specific CAN ID.

---

### 1) PID request/response

Use `request_pid` for typical OBD-II reads.

```python
from simple_elm import ELM327, OBDPid

with ELM327(port="COM9", baudrate=38400, timeout=1.0) as elm:
    elm.initialize(protocol=6, headers=True)

    # Request Mode 01 PID 0C (engine RPM) from functional header 7DF.
    response = elm.request_pid(OBDPid.ENGINE_RPM, header="7DF")
    print("PID response:", response)
```

Example response (adapter-dependent formatting):

```text
7E8 04 41 0C 1A F8
```

---

### 2) Send CAN data

Use `send_can` when you want to explicitly set a header and send raw payload data.
(`send_data` is the lower-level equivalent.)

```python
from simple_elm import ELM327

with ELM327(port="COM9", baudrate=38400, timeout=1.0) as elm:
    elm.initialize(protocol=6, headers=True)

    # Send 010C to ECU header 7E0.
    response = elm.send_can(header="7E0", data="010C")
    print("CAN response:", response)
```

---

### 3) CAN monitor (stream CAN ID)

Use `start_monitor_can_id` to continuously stream frames for one responder ID into
the built-in thread-safe queue (`CANBuffer`).

```python
import time
from simple_elm import ELM327

with ELM327(port="COM9", baudrate=38400, timeout=1.0) as elm:
    elm.initialize(protocol=6, headers=True)

    buffer = elm.start_monitor_can_id("60D")
    time.sleep(5.0)  # collect frames for a short window
    elm.stop_monitor()

    while not buffer.empty():
        frame = buffer.get_nowait()
        print(frame.can_id, frame.data, frame.timestamp)
```

Use `start_monitor_all()` if you want all traffic instead of filtering by one ID.
