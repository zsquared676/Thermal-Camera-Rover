"""Pico 2 W thermal rover controller.

Hardware map:
  MLX90640: SDA GP6, SCL GP7, 3V3, GND
  TB6612: PWMA GP8, AIN1 GP9, AIN2 GP10,
           PWMB GP11, BIN1 GP12, BIN2 GP13
  Pan servo signal: GP16
  Tilt servo signal: GP17
  HC-SR04: TRIG GP18, ECHO GP19 through a 1k/2k divider

The Pico creates an open network named Pico-Rover-Open. Connect to it and
open http://192.168.4.1/. Motors have a command-loss watchdog. Servos must
use a separate regulated supply; never power them from Pico 3V3.
"""

import gc
import network
import socket
import time
from machine import Pin, PWM, SoftI2C, time_pulse_us

from mlx90640 import MLX90640, RefreshRate, init_float_array


# ---------------- Network ----------------
AP_NAME = "Pico-Rover-Open"


# ---------------- Safety and speed ----------------
DEFAULT_SPEED_PERCENT = 35
MOTOR_TIMEOUT_MS = 1000
SERVO_TIMEOUT_MS = 800
LEFT_DIRECTION = 1
RIGHT_DIRECTION = 1

# Keep this False until the HC-SR04 power fault has been resolved.
ULTRASONIC_ENABLED = False


# ---------------- Motor driver ----------------
pwma = PWM(Pin(8))
ain1 = Pin(9, Pin.OUT)
ain2 = Pin(10, Pin.OUT)
pwmb = PWM(Pin(11))
bin1 = Pin(12, Pin.OUT)
bin2 = Pin(13, Pin.OUT)
pwma.freq(1000)
pwmb.freq(1000)

speed_percent = DEFAULT_SPEED_PERCENT
motor_moving = False
last_motor_command = time.ticks_ms()


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def set_motor(pwm, input_1, input_2, signed_percent):
    signed_percent = clamp(int(signed_percent), -100, 100)
    if signed_percent > 0:
        input_1.value(1)
        input_2.value(0)
    elif signed_percent < 0:
        input_1.value(0)
        input_2.value(1)
    else:
        input_1.value(0)
        input_2.value(0)
    pwm.duty_u16(int(abs(signed_percent) * 65535 / 100))


def drive(left_percent, right_percent):
    set_motor(pwma, ain1, ain2, left_percent * LEFT_DIRECTION)
    set_motor(pwmb, bin1, bin2, right_percent * RIGHT_DIRECTION)


def stop_motors():
    global motor_moving
    drive(0, 0)
    motor_moving = False


def motor_command(command):
    global motor_moving, last_motor_command
    power = speed_percent
    if command == "forward":
        drive(power, power)
        motor_moving = True
    elif command == "reverse":
        drive(-power, -power)
        motor_moving = True
    elif command == "left":
        drive(-power, power)
        motor_moving = True
    elif command == "right":
        drive(power, -power)
        motor_moving = True
    else:
        stop_motors()
    last_motor_command = time.ticks_ms()


# ---------------- Pan and tilt timed-step servos ----------------
pan = PWM(Pin(16))
tilt = PWM(Pin(17))
pan.freq(50)
tilt.freq(50)

# These servos are behaving as continuous-rotation units. Each command applies
# a slow directional pulse briefly, then the Pico sends neutral locally. Adjust
# SERVO_STEP_MS if one tap rotates more or less than approximately 5 degrees.
SERVO_STEP_DEGREES = 5
SERVO_STEP_MS = 40
PAN_LEFT_US = 1350
PAN_RIGHT_US = 1650
TILT_UP_US = 1350
TILT_DOWN_US = 1650

pan_position = 90
tilt_position = 90

servos_active = False
last_servo_command = time.ticks_ms()


def servo_pulse(servo, pulse_us):
    servo.duty_u16(int(pulse_us * 65535 / 20000))


def apply_servo_positions():
    pan.duty_u16(0)
    tilt.duty_u16(0)


def stop_servos():
    global servos_active
    # Remove the control pulses completely. This avoids neutral-point creep
    # that can occur when a continuous servo does not stop at exactly 1500 us.
    pan.duty_u16(0)
    tilt.duty_u16(0)
    servos_active = False


def timed_servo_step(servo, direction_pulse_us):
    global servos_active
    servos_active = True
    try:
        servo_pulse(servo, direction_pulse_us)
        time.sleep_ms(SERVO_STEP_MS)
    finally:
        servo.duty_u16(0)
        servos_active = False


def servo_command(command):
    global pan_position, tilt_position, servos_active, last_servo_command
    if command == "pan_left":
        timed_servo_step(pan, PAN_LEFT_US)
        pan_position -= SERVO_STEP_DEGREES
    elif command == "pan_right":
        timed_servo_step(pan, PAN_RIGHT_US)
        pan_position += SERVO_STEP_DEGREES
    elif command == "tilt_up":
        timed_servo_step(tilt, TILT_UP_US)
        tilt_position -= SERVO_STEP_DEGREES
    elif command == "tilt_down":
        timed_servo_step(tilt, TILT_DOWN_US)
        tilt_position += SERVO_STEP_DEGREES
    else:
        stop_servos()
    last_servo_command = time.ticks_ms()


def servo_json():
    return '{"ok":true,"pan":%d,"tilt":%d}' % (
        pan_position,
        tilt_position,
    )


def emergency_stop():
    stop_motors()
    stop_servos()


# ---------------- HC-SR04 ultrasonic sensor ----------------
ultrasonic_trigger = Pin(18, Pin.OUT, value=0)
ultrasonic_echo = Pin(19, Pin.IN)
distance_cm = None
distance_error = "Waiting for HC-SR04"
last_distance_measurement = time.ticks_ms()


def update_distance():
    global distance_cm, distance_error, last_distance_measurement
    last_distance_measurement = time.ticks_ms()
    try:
        ultrasonic_trigger.value(0)
        time.sleep_us(2)
        ultrasonic_trigger.value(1)
        time.sleep_us(10)
        ultrasonic_trigger.value(0)

        duration_us = time_pulse_us(ultrasonic_echo, 1, 30000)
        if duration_us < 0:
            raise RuntimeError("Echo timeout")

        measured = duration_us * 0.0343 / 2
        if measured < 2 or measured > 400:
            raise RuntimeError("Out of range")

        distance_cm = measured
        distance_error = ""
    except Exception as error:
        distance_cm = None
        distance_error = str(error)


def distance_json():
    if not ULTRASONIC_ENABLED:
        return '{"ok":false,"error":"Ultrasonic disabled"}'
    if distance_cm is None:
        return '{"ok":false,"error":"' + json_escape(distance_error) + '"}'
    return '{"ok":true,"cm":%.1f}' % distance_cm


# ---------------- Thermal camera (safe on-demand capture) ----------------
thermal_camera = None
thermal_frame = init_float_array(768)


def json_escape(value):
    return str(value).replace("\\", "\\\\").replace('"', '\\"')


def capture_thermal_json():
    global thermal_camera

    if motor_moving:
        return '{"ok":false,"error":"Stop rover before thermal capture"}'

    try:
        if thermal_camera is None:
            i2c = SoftI2C(sda=Pin(6), scl=Pin(7), freq=100000, timeout=50000)
            if 0x33 not in i2c.scan():
                raise RuntimeError("MLX90640 not found at 0x33")
            thermal_camera = MLX90640(i2c)
            thermal_camera.refresh_rate = RefreshRate.REFRESH_2_HZ
            print("Thermal camera ready at 0x33")

        gc.collect()
        thermal_camera.get_frame(thermal_frame)
        thermal_camera.get_frame(thermal_frame)

        minimum = min(thermal_frame)
        maximum = max(thermal_frame)
        center = thermal_frame[12 * 32 + 16]
        pixels = ",".join("{:.1f}".format(value) for value in thermal_frame)
        return (
            '{"ok":true,"min":%.1f,"max":%.1f,"center":%.1f,'
            '"pixels":[%s]}' % (minimum, maximum, center, pixels)
        )

    except Exception as error:
        message = json_escape(error)
        print("Thermal camera error:", message)
        thermal_camera = None
        return '{"ok":false,"error":"' + message + '"}'


# ---------------- Direct Wi-Fi access point ----------------
def start_access_point():
    try:
        station = network.WLAN(network.WLAN.IF_STA)
        ap = network.WLAN(network.WLAN.IF_AP)
    except AttributeError:
        station = network.WLAN(network.STA_IF)
        ap = network.WLAN(network.AP_IF)

    station.active(False)
    ap.active(False)
    time.sleep_ms(250)
    ap.config(ssid=AP_NAME, security=0, channel=6)
    ap.active(True)

    start = time.ticks_ms()
    while not ap.active():
        if time.ticks_diff(time.ticks_ms(), start) > 10000:
            raise RuntimeError("Could not start Pico-Rover access point")
        time.sleep_ms(100)

    ip_address = ap.ifconfig()[0]
    print("Direct rover Wi-Fi ready")
    print("Network name:", AP_NAME)
    print("No password is required")
    print("Open http://" + ip_address)


PAGE = """<!doctype html>
<html><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<meta name="theme-color" content="#111827">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta name="mobile-web-app-capable" content="yes">
<title>Thermal Rover</title>
<style>
*{box-sizing:border-box}
html{background:#111827;overflow-x:hidden}
body{font-family:Arial,sans-serif;text-align:center;background:#111827;color:white;margin:0;padding:14px 14px 100px;min-height:100vh;overflow-x:hidden;overflow-y:scroll;touch-action:auto;-webkit-overflow-scrolling:touch}
h1,h2{margin:8px}.status{color:#9ca3af;margin:5px}.grid{display:flex;flex-wrap:wrap;gap:18px;justify-content:center}
.mobileTabs{display:none}
.panel{background:#1f2937;border-radius:16px;padding:14px;width:360px;max-width:100%;min-width:0}.pad{display:grid;grid-template-columns:repeat(3,minmax(0,86px));gap:10px;justify-content:center}
button{height:70px;border:0;border-radius:14px;font-size:27px;background:#374151;color:white;touch-action:manipulation;-webkit-user-select:none;user-select:none}button:active{background:#2563eb}
.stop{position:fixed;left:50%;bottom:12px;transform:translateX(-50%);z-index:20;background:#b91c1c;font-size:20px;width:278px;max-width:calc(100vw - 28px);box-shadow:0 4px 18px #000}
canvas{width:320px;max-width:100%;height:auto;aspect-ratio:4/3;image-rendering:pixelated;background:#000;border-radius:8px}
input{width:240px;max-width:100%;touch-action:pan-x}.small{font-size:14px;color:#d1d5db}
@media(max-width:600px){body{padding:8px 8px 94px;overflow-y:scroll;touch-action:auto}h1{font-size:25px}.mobileTabs{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin:8px auto 12px;max-width:420px}.tabButton{height:46px;font-size:15px;padding:4px}.tabButton.active{background:#2563eb}.grid{display:block}.panel{display:none;width:100%;padding:12px;margin:0 auto}.panel.active{display:block}.pad{grid-template-columns:repeat(3,minmax(0,82px))}button{height:62px}}
</style></head><body>
<h1>Thermal Rover</h1><div id="status" class="status">Connected</div>
<nav class="mobileTabs">
 <button class="tabButton active" data-panel="drivePanel">Drive</button>
 <button class="tabButton" data-panel="thermalPanel">Thermal</button>
 <button class="tabButton" data-panel="servoPanel">Camera Aim</button>
</nav>
<div class="grid">
 <section class="panel active" id="drivePanel"><h2>Drive</h2><div class="pad">
  <span></span><button data-motor="forward">&#9650;</button><span></span>
  <button data-motor="left">&#9664;</button><button data-motor="stop">&#9632;</button><button data-motor="right">&#9654;</button>
  <span></span><button data-motor="reverse">&#9660;</button><span></span>
 </div><p>Speed: <span id="speedValue">35</span>%</p><input id="speed" type="range" min="25" max="80" value="35"></section>
 <section class="panel" id="thermalPanel"><h2>Thermal Camera</h2><canvas id="thermal" width="32" height="24"></canvas>
  <p id="temps" class="small">Press Refresh while the rover is stopped.</p>
  <button id="refreshThermal" style="font-size:18px;width:240px">Refresh Thermal</button></section>
 <section class="panel" id="servoPanel"><h2>Pan / Tilt</h2><div class="pad">
  <span></span><button data-servo="tilt_up">&#9650;</button><span></span>
  <button data-servo="pan_left">&#9664;</button><button data-servo="stop" style="font-size:18px">STOP</button><button data-servo="pan_right">&#9654;</button>
  <span></span><button data-servo="tilt_down">&#9660;</button><span></span>
 </div><p id="servoPosition" class="small">Estimated pan 90&deg; | tilt 90&deg;</p>
  <p class="small">Each tap makes one timed step and then automatically sends STOP.</p>
  <p class="small">Servos require a separate regulated 6 V supply.</p>
  <h2>Obstacle Distance</h2><p id="distance" class="small">Waiting for HC-SR04...</p></section>
</div><button class="stop" id="allStop">EMERGENCY STOP</button>
<script>
let motorTimer=null,motorCmd="stop",thermalBusy=false;
const statusText=document.getElementById("status");
document.querySelectorAll(".tabButton").forEach(b=>b.addEventListener("click",()=>{document.querySelectorAll(".tabButton").forEach(x=>x.classList.remove("active"));document.querySelectorAll(".panel").forEach(x=>x.classList.remove("active"));b.classList.add("active");document.getElementById(b.dataset.panel).classList.add("active");window.scrollTo(0,0)}));
function send(path){return fetch(path,{cache:"no-store"}).catch(()=>{statusText.textContent="Connection lost - rover should auto-stop"})}
function motorStart(cmd){if(motorCmd===cmd&&motorTimer!==null)return;motorCmd=cmd;statusText.textContent=cmd;send("/move?cmd="+cmd);clearInterval(motorTimer);motorTimer=setInterval(()=>send("/move?cmd="+motorCmd),300)}
function motorStop(){const wasActive=motorCmd!=="stop"||motorTimer!==null;motorCmd="stop";clearInterval(motorTimer);motorTimer=null;if(wasActive){statusText.textContent="Stopped";send("/move?cmd=stop")}}
function sendServo(cmd){return fetch("/servo?cmd="+cmd,{cache:"no-store"}).then(r=>r.json()).then(d=>{if(d.ok)document.getElementById("servoPosition").textContent=`Estimated pan ${d.pan}\u00b0 | tilt ${d.tilt}\u00b0`}).catch(()=>{statusText.textContent="Connection lost - rover should auto-stop"})}
function bindHold(selector,start,stop){document.querySelectorAll(selector).forEach(b=>{const cmd=b.dataset.motor;
 b.addEventListener("pointerdown",e=>{e.preventDefault();b.setPointerCapture(e.pointerId);cmd==="stop"?stop():start(cmd)});
 ["pointerup","pointercancel","lostpointercapture"].forEach(x=>b.addEventListener(x,stop));});}
bindHold("button[data-motor]",motorStart,motorStop);
document.querySelectorAll("button[data-servo]").forEach(b=>b.addEventListener("click",e=>{e.preventDefault();sendServo(b.dataset.servo)}));
document.getElementById("allStop").onclick=()=>{motorStop();send("/stop")};
const speed=document.getElementById("speed"),speedValue=document.getElementById("speedValue");
speed.oninput=()=>speedValue.textContent=speed.value;speed.onchange=()=>send("/speed?value="+speed.value);
window.addEventListener("blur",motorStop);
const canvas=document.getElementById("thermal"),ctx=canvas.getContext("2d"),img=ctx.createImageData(32,24);
function color(v,min,max){let x=Math.max(0,Math.min(1,(v-min)/Math.max(.1,max-min)));return [Math.floor(255*Math.min(1,2*x)),Math.floor(255*Math.max(0,2*x-1)),Math.floor(255*Math.max(0,1-2*x))]}
async function thermal(){if(thermalBusy||motorCmd!=="stop")return;thermalBusy=true;document.getElementById("temps").textContent="Capturing thermal frame...";try{const d=await fetch("/thermal",{cache:"no-store"}).then(r=>r.json());if(!d.ok){document.getElementById("temps").textContent=d.error;return}
 for(let i=0;i<768;i++){const c=color(d.pixels[i],d.min,d.max),j=i*4;img.data[j]=c[0];img.data[j+1]=c[1];img.data[j+2]=c[2];img.data[j+3]=255}ctx.putImageData(img,0,0);
 document.getElementById("temps").textContent=`Min ${d.min.toFixed(1)} C | Center ${d.center.toFixed(1)} C | Max ${d.max.toFixed(1)} C`;}catch(e){document.getElementById("temps").textContent="Thermal update unavailable"}finally{thermalBusy=false}}
document.getElementById("refreshThermal").onclick=thermal;
async function distance(){try{const d=await fetch("/distance",{cache:"no-store"}).then(r=>r.json());document.getElementById("distance").textContent=d.ok?`${d.cm.toFixed(1)} cm`:d.error}catch(e){document.getElementById("distance").textContent="Distance update unavailable"}}
distance();
</script></body></html>"""


def query_value(path, key):
    marker = key + "="
    if marker not in path:
        return None
    return path.split(marker, 1)[1].split("&", 1)[0]


def send_response(client, body, content_type="text/plain; charset=utf-8", status="200 OK"):
    if isinstance(body, str):
        body = body.encode()
    header = (
        "HTTP/1.1 " + status + "\r\n"
        "Content-Type: " + content_type + "\r\n"
        "Cache-Control: no-store\r\n"
        "Content-Length: " + str(len(body)) + "\r\n"
        "Connection: close\r\n\r\n"
    )
    client.sendall(header.encode())
    client.sendall(body)


def start_server():
    global speed_percent
    address = socket.getaddrinfo("0.0.0.0", 80)[0][-1]
    server = socket.socket()
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(address)
    server.listen(4)
    server.settimeout(0.08)
    print("Integrated thermal rover controller ready")

    while True:
        now = time.ticks_ms()
        if motor_moving and time.ticks_diff(now, last_motor_command) > MOTOR_TIMEOUT_MS:
            print("Motor command timeout: stopping")
            stop_motors()
        if servos_active and time.ticks_diff(now, last_servo_command) > SERVO_TIMEOUT_MS:
            stop_servos()
        if ULTRASONIC_ENABLED and time.ticks_diff(now, last_distance_measurement) > 250:
            update_distance()

        client = None
        try:
            client, _ = server.accept()
            request = client.recv(1024)
            if not request:
                continue
            first_line = request.split(b"\r\n", 1)[0].decode()
            parts = first_line.split(" ")
            path = parts[1] if len(parts) > 1 else "/"

            if path.startswith("/move?"):
                motor_command(query_value(path, "cmd") or "stop")
                send_response(client, "OK")
            elif path.startswith("/servo?"):
                servo_command(query_value(path, "cmd") or "stop")
                send_response(client, servo_json(), "application/json; charset=utf-8")
            elif path.startswith("/speed?"):
                requested = query_value(path, "value")
                if requested is not None:
                    speed_percent = clamp(int(requested), 25, 80)
                    stop_motors()
                send_response(client, str(speed_percent))
            elif path == "/thermal":
                send_response(client, capture_thermal_json(), "application/json; charset=utf-8")
            elif path == "/distance":
                send_response(client, distance_json(), "application/json; charset=utf-8")
            elif path == "/stop":
                emergency_stop()
                send_response(client, "STOPPED")
            else:
                send_response(client, PAGE, "text/html; charset=utf-8")

        except OSError:
            pass
        except Exception as error:
            print("Request error:", error)
            emergency_stop()
        finally:
            if client is not None:
                try:
                    client.close()
                except Exception:
                    pass


apply_servo_positions()
emergency_stop()
try:
    start_access_point()
    start_server()
finally:
    emergency_stop()
