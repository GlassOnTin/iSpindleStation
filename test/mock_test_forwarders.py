#!/usr/bin/env python3
"""
Empirical Mock Test Suite for all 12 iSpindleStation forwarder services (0..11),
testing both On-Receipt iSpindel forwarding and Periodic Relay Sensor uploading
live from the ESP8266 against a local Mock HTTP Server and Mock MQTT v3.1.1 Broker.
"""

import json
import socket
import struct
import threading
import time
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, HTTPServer

ESP_IP = "192.168.0.146"
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as _s:
    _s.connect((ESP_IP, 80))
    HOST_IP = _s.getsockname()[0]
HTTP_PORT = 18080
MQTT_PORT = 18830

captured_http = []
http_cond = threading.Condition()

captured_mqtt = {
    "connects": [],
    "publishes": [],
}
mqtt_cond = threading.Condition()


class MockHTTPHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        raw_body = self.rfile.read(length).decode("utf-8", errors="replace")
        try:
            parsed_json = json.loads(raw_body)
        except Exception:
            parsed_json = None

        rec = {
            "method": "POST",
            "path": self.path,
            "headers": {k: v for k, v in self.headers.items()},
            "raw_body": raw_body,
            "json": parsed_json,
        }
        with http_cond:
            captured_http.append(rec)
            http_cond.notify_all()

        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Connection", "close")
        self.end_headers()
        self.wfile.write(b'{"result":"ok"}\n')

    def log_message(self, fmt, *args):
        pass


def decode_mqtt_varint(sock):
    multiplier = 1
    value = 0
    while True:
        b = sock.recv(1)
        if not b:
            return None
        encoded = b[0]
        value += (encoded & 127) * multiplier
        if (encoded & 128) == 0:
            break
        multiplier *= 128
    return value


def recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def read_utf8(data, offset):
    strlen = struct.unpack("!H", data[offset : offset + 2])[0]
    s = data[offset + 2 : offset + 2 + strlen].decode("utf-8", errors="replace")
    return s, offset + 2 + strlen


def handle_mqtt_client(conn):
    conn.settimeout(5.0)
    try:
        while True:
            hdr = conn.recv(1)
            if not hdr:
                break
            pkt_type = (hdr[0] >> 4) & 0x0F
            flags = hdr[0] & 0x0F
            rem_len = decode_mqtt_varint(conn)
            if rem_len is None:
                break
            payload = recv_exact(conn, rem_len) if rem_len > 0 else b""
            if payload is None:
                break

            if pkt_type == 1:  # CONNECT
                proto_name, off = read_utf8(payload, 0)
                proto_level = payload[off]
                conn_flags = payload[off + 1]
                keepalive = struct.unpack("!H", payload[off + 2 : off + 4])[0]
                off += 4
                client_id, off = read_utf8(payload, off)
                username = None
                password = None
                if conn_flags & 0x80:
                    username, off = read_utf8(payload, off)
                if conn_flags & 0x40:
                    password, off = read_utf8(payload, off)
                with mqtt_cond:
                    captured_mqtt["connects"].append(
                        {
                            "proto": proto_name,
                            "level": proto_level,
                            "client_id": client_id,
                            "username": username,
                            "password": password,
                            "keepalive": keepalive,
                        }
                    )
                    mqtt_cond.notify_all()
                # Send CONNACK (0x20, len=2, flags=0, rc=0)
                conn.sendall(b"\x20\x02\x00\x00")

            elif pkt_type == 3:  # PUBLISH
                retain = bool(flags & 0x01)
                qos = (flags >> 1) & 0x03
                topic, off = read_utf8(payload, 0)
                if qos > 0:
                    pkt_id = struct.unpack("!H", payload[off : off + 2])[0]
                    off += 2
                msg_bytes = payload[off:]
                msg_str = msg_bytes.decode("utf-8", errors="replace")
                with mqtt_cond:
                    captured_mqtt["publishes"].append(
                        {
                            "topic": topic,
                            "payload": msg_str,
                            "retain": retain,
                            "qos": qos,
                        }
                    )
                    mqtt_cond.notify_all()
                if qos == 1:
                    conn.sendall(struct.pack("!BBH", 0x40, 2, pkt_id))

            elif pkt_type == 12:  # PINGREQ
                conn.sendall(b"\xD0\x00")

            elif pkt_type == 14:  # DISCONNECT
                break
    except Exception:
        pass
    finally:
        conn.close()


def mqtt_broker_loop(srv_sock):
    while True:
        try:
            conn, _ = srv_sock.accept()
            threading.Thread(target=handle_mqtt_client, args=(conn,), daemon=True).start()
        except Exception:
            break


def call_test_forward(params, body_dict=None):
    qs = urllib.parse.urlencode(params)
    url = f"http://{ESP_IP}/api/test_forward?{qs}"
    data = json.dumps(body_dict).encode("utf-8") if body_dict is not None else b""
    hdrs = {"Connection": "close"}
    if body_dict is not None:
        hdrs["Content-Type"] = "application/json"

    for attempt in range(4):
        with http_cond:
            captured_http.clear()
        with mqtt_cond:
            captured_mqtt["connects"].clear()
            captured_mqtt["publishes"].clear()
        try:
            req = urllib.request.Request(
                url,
                data=data if body_dict is not None else b"",
                headers=hdrs,
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=12) as resp:
                return json.loads(resp.read().decode("utf-8"))
        except Exception as e:
            if attempt == 3:
                raise
            time.sleep(0.5)


def wait_http(timeout=3.0):
    deadline = time.time() + timeout
    with http_cond:
        while not captured_http and time.time() < deadline:
            http_cond.wait(timeout=0.2)
        return list(captured_http)


if __name__ == "__main__":
    http_srv = HTTPServer((HOST_IP, HTTP_PORT), MockHTTPHandler)
    threading.Thread(target=http_srv.serve_forever, daemon=True).start()

    mqtt_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    mqtt_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    mqtt_sock.bind((HOST_IP, MQTT_PORT))
    mqtt_sock.listen(8)
    threading.Thread(target=mqtt_broker_loop, args=(mqtt_sock,), daemon=True).start()

    target = f"{HOST_IP}:{HTTP_PORT}"
    sample_1 = {
        "name": "1-PaleAle",
        "ID": 771122,
        "angle": 42.75,
        "temperature": 19.5,
        "temp_units": "C",
        "battery": 4.12,
        "gravity": 1.052,
        "interval": 900,
        "RSSI": -58,
    }
    sample_2 = {
        "name": "2-Lager",
        "ID": 883344,
        "angle": 31.20,
        "temperature": 11.8,
        "temp_units": "C",
        "battery": 3.98,
        "gravity": 1.024,
        "interval": 900,
        "RSSI": -64,
    }

    results = []

    def check(test_name, cond, detail=""):
        status = "PASS" if cond else "FAIL"
        results.append((test_name, cond, detail))
        print(f"[{status}] {test_name}: {detail}")
        if not cond:
            raise AssertionError(f"{test_name} failed: {detail}")

    # ---- PART 1: ON-RECEIPT FORWARDERS (Services 0..11) ----

    # Service 0: UbiDots on-receipt
    r = call_test_forward(
        {"target": target, "mode": "receipt", "service": 0, "TOKEN": "BBFF-ubidots123"},
        sample_1,
    )
    reqs = wait_http()
    check(
        "0. UbiDots On-Receipt",
        r["ok"]
        and len(reqs) == 1
        and reqs[0]["headers"].get("Host") == "industrial.api.ubidots.com"
        and reqs[0]["headers"].get("User-Agent") == "ESP8266"
        and reqs[0]["headers"].get("Accept") == "*/*"
        and reqs[0]["path"] == "/api/v1.6/devices/1-PaleAle?token=BBFF-ubidots123"
        and reqs[0]["json"]["gravity"] == 1.052,
        f"Host={reqs[0]['headers'].get('Host')} UA={reqs[0]['headers'].get('User-Agent')} Path={reqs[0]['path']}",
    )

    # Service 1: Brewfather on-receipt
    r = call_test_forward(
        {"target": target, "mode": "receipt", "service": 1, "uri": "/ispindel?id=bf999"},
        sample_1,
    )
    reqs = wait_http()
    check(
        "1. Brewfather On-Receipt",
        r["ok"]
        and len(reqs) == 1
        and reqs[0]["headers"].get("Host") == "log.brewfather.net"
        and reqs[0]["headers"].get("User-Agent") == "iSpindel"
        and reqs[0]["path"] == "/ispindel?id=bf999"
        and reqs[0]["json"]["name"] == "1-PaleAle",
        f"Host={reqs[0]['headers'].get('Host')} UA={reqs[0]['headers'].get('User-Agent')} Path={reqs[0]['path']}",
    )

    # Service 2: BrewSpy on-receipt (default path /api/ispindel + token injected)
    r = call_test_forward(
        {"target": target, "mode": "receipt", "service": 2, "uri": "/", "TOKEN": "bspy-tok-42"},
        sample_1,
    )
    reqs = wait_http()
    check(
        "2. BrewSpy On-Receipt",
        r["ok"]
        and len(reqs) == 1
        and reqs[0]["headers"].get("Host") == "brew-spy.com"
        and reqs[0]["headers"].get("User-Agent") == "iSpindel"
        and reqs[0]["path"] == "/api/ispindel"
        and reqs[0]["json"].get("token") == "bspy-tok-42"
        and reqs[0]["json"]["gravity"] == 1.052,
        f"Host={reqs[0]['headers'].get('Host')} Path={reqs[0]['path']} token={reqs[0]['json'].get('token')}",
    )

    # Service 3: Monitor.Beer on-receipt (token injected)
    r = call_test_forward(
        {"target": target, "mode": "receipt", "service": 3, "uri": "/api/v1/ispindel", "TOKEN": "mb-tok-77"},
        sample_1,
    )
    reqs = wait_http()
    check(
        "3. Monitor.Beer On-Receipt",
        r["ok"]
        and len(reqs) == 1
        and reqs[0]["headers"].get("Host") == "Monitor.beer"
        and reqs[0]["headers"].get("User-Agent") == "iSpindel"
        and reqs[0]["path"] == "/api/v1/ispindel"
        and reqs[0]["json"].get("token") == "mb-tok-77",
        f"Host={reqs[0]['headers'].get('Host')} Path={reqs[0]['path']} token={reqs[0]['json'].get('token')}",
    )

    # Service 4: Grainfather on-receipt (Slot 1 '1-*' -> grainfather_uri1, Slot 2 '2-*' -> grainfather_uri2)
    r1 = call_test_forward(
        {
            "target": target,
            "mode": "receipt",
            "service": 4,
            "grainfather_uri1": "/iot/gf-slot1/ispindel",
            "grainfather_uri2": "/iot/gf-slot2/ispindel",
        },
        sample_1,
    )
    reqs1 = wait_http()
    r2 = call_test_forward(
        {
            "target": target,
            "mode": "receipt",
            "service": 4,
            "grainfather_uri1": "/iot/gf-slot1/ispindel",
            "grainfather_uri2": "/iot/gf-slot2/ispindel",
        },
        sample_2,
    )
    reqs2 = wait_http()
    check(
        "4. Grainfather On-Receipt (Slot 1 & Slot 2 routing)",
        r1["ok"]
        and r2["ok"]
        and reqs1[0]["headers"].get("Host") == "community.grainfather.com"
        and reqs1[0]["path"] == "/iot/gf-slot1/ispindel"
        and reqs2[0]["path"] == "/iot/gf-slot2/ispindel",
        f"Slot1={reqs1[0]['path']} Slot2={reqs2[0]['path']}",
    )

    # Service 5: BrewPiless on-receipt (default 192.168.88.55, rewrites name -> iSpindel000)
    r = call_test_forward(
        {"target": target, "mode": "receipt", "service": 5, "server_address": ""},
        sample_1,
    )
    reqs = wait_http()
    check(
        "5. BrewPiless On-Receipt",
        r["ok"]
        and len(reqs) == 1
        and reqs[0]["headers"].get("Host") == "192.168.88.55"
        and reqs[0]["path"] == "/gravity"
        and reqs[0]["json"].get("name") == "iSpindel000",
        f"Host={reqs[0]['headers'].get('Host')} Path={reqs[0]['path']} name={reqs[0]['json'].get('name')}",
    )

    # Service 6: Little Bock on-receipt (Slot 1 '1-*' -> littlebock_uri1, Slot 2 '2-*' -> littlebock_uri2)
    r1 = call_test_forward(
        {
            "target": target,
            "mode": "receipt",
            "service": 6,
            "littlebock_uri1": "/api/log/ispindle/lb1",
            "littlebock_uri2": "/api/log/ispindle/lb2",
        },
        sample_1,
    )
    reqs1 = wait_http()
    r2 = call_test_forward(
        {
            "target": target,
            "mode": "receipt",
            "service": 6,
            "littlebock_uri1": "/api/log/ispindle/lb1",
            "littlebock_uri2": "/api/log/ispindle/lb2",
        },
        sample_2,
    )
    reqs2 = wait_http()
    check(
        "6. Little Bock On-Receipt (Slot 1 & Slot 2 routing)",
        r1["ok"]
        and r2["ok"]
        and reqs1[0]["headers"].get("Host") == "www.littlebock.fr"
        and reqs1[0]["path"] == "/api/log/ispindle/lb1"
        and reqs2[0]["path"] == "/api/log/ispindle/lb2",
        f"Slot1={reqs1[0]['path']} Slot2={reqs2[0]['path']}",
    )

    # Service 7: Brewers Friend on-receipt
    r = call_test_forward(
        {"target": target, "mode": "receipt", "service": 7, "uri": "/ispindel/bfriendKey123"},
        sample_1,
    )
    reqs = wait_http()
    check(
        "7. Brewers Friend On-Receipt",
        r["ok"]
        and len(reqs) == 1
        and reqs[0]["headers"].get("Host") == "log.brewersfriend.com"
        and reqs[0]["headers"].get("User-Agent") == "iSpindel"
        and reqs[0]["path"] == "/ispindel/bfriendKey123",
        f"Host={reqs[0]['headers'].get('Host')} Path={reqs[0]['path']}",
    )

    # Service 8: BrewBench on-receipt (concatenates uri + TOKEN)
    r = call_test_forward(
        {"target": target, "mode": "receipt", "service": 8, "uri": "/ispindel/", "TOKEN": "bb-secret-99"},
        sample_1,
    )
    reqs = wait_http()
    check(
        "8. BrewBench On-Receipt",
        r["ok"]
        and len(reqs) == 1
        and reqs[0]["headers"].get("Host") == "sensor.brewbench.co"
        and reqs[0]["path"] == "/ispindel/bb-secret-99",
        f"Host={reqs[0]['headers'].get('Host')} Path={reqs[0]['path']}",
    )

    # Service 9: MQTT + 17 Home Assistant Auto-Discovery sensors + Slot 1 & Slot 2 state topics
    r1 = call_test_forward(
        {
            "mode": "receipt",
            "service": 9,
            "uri": f"{HOST_IP}:{MQTT_PORT}",
            "ap_ssid": "nautilis-test",
            "mqtt_user": "brewer",
            "mqtt_pass": "hops123",
            "ispindel_num": 2,
            "unit_1": 1,
            "temp_yesno": 2,
            "temp_unit": 1,
            "pressure_unit": 1,
            "relay_temp": 18.25,
            "relay_bar": 0.85,
            "OG": 1.055,
        },
        sample_1,
    )
    time.sleep(0.3)
    with mqtt_cond:
        conns = list(captured_mqtt["connects"])
        pubs = list(captured_mqtt["publishes"])
    ha_topics = [p["topic"] for p in pubs if p["topic"].startswith("homeassistant/sensor/")]
    state_map = {p["topic"]: p["payload"] for p in pubs if not p["topic"].startswith("homeassistant/")}
    check(
        "9a. MQTT On-Receipt (Slot 1 + 17 HA Discovery + Auth)",
        r1["ok"]
        and len(conns) == 1
        and conns[0]["username"] == "brewer"
        and conns[0]["password"] == "hops123"
        and len(ha_topics) == 17
        and state_map.get("nautilis-test/I1_gravity") == "1.052"
        and state_map.get("nautilis-test/I1_temp") == "19.50"
        and state_map.get("nautilis-test/I1_bat") == "4.12"
        and state_map.get("nautilis-test/I1_OG1") == "1.055"
        and state_map.get("nautilis-test/temp") == "18.25"
        and state_map.get("nautilis-test/pressure") == "0.85",
        f"ha_count={len(ha_topics)} I1_gravity={state_map.get('nautilis-test/I1_gravity')} I1_OG1={state_map.get('nautilis-test/I1_OG1')}",
    )

    # Service 9b: MQTT Slot 2 with Plato (unit_2=2) and Fahrenheit (temp_unit=2)
    r2 = call_test_forward(
        {
            "mode": "receipt",
            "service": 9,
            "uri": f"{HOST_IP}:{MQTT_PORT}",
            "ap_ssid": "nautilis-test",
            "ispindel_num": 2,
            "unit_2": 2,
            "temp_unit": 2,
            "OG": 1.048,
        },
        sample_2,
    )
    time.sleep(0.3)
    with mqtt_cond:
        pubs2 = list(captured_mqtt["publishes"])
    state_map2 = {p["topic"]: p["payload"] for p in pubs2 if not p["topic"].startswith("homeassistant/")}
    check(
        "9b. MQTT On-Receipt (Slot 2 in Plato & Fahrenheit)",
        r2["ok"]
        and state_map2.get("nautilis-test/I2_gravity") in ("6.07", "6.08")
        and state_map2.get("nautilis-test/I2_temp") == "53.24"
        and state_map2.get("nautilis-test/I2_OG2") == "11.91",
        f"I2_gravity(°P)={state_map2.get('nautilis-test/I2_gravity')} I2_temp(°F)={state_map2.get('nautilis-test/I2_temp')} I2_OG2(°P)={state_map2.get('nautilis-test/I2_OG2')}",
    )

    # Service 10: Generic HTTP on-receipt (direct connection to server_address + token injected)
    r = call_test_forward(
        {
            "mode": "receipt",
            "service": 10,
            "server_address": target,
            "uri": "/custom/ingest",
            "TOKEN": "generic-tok-10",
        },
        sample_1,
    )
    reqs = wait_http()
    check(
        "10. Generic HTTP On-Receipt",
        r["ok"]
        and len(reqs) == 1
        and reqs[0]["headers"].get("Host") == HOST_IP
        and reqs[0]["headers"].get("User-Agent") == "iSpindel"
        and reqs[0]["path"] == "/custom/ingest"
        and reqs[0]["json"].get("token") == "generic-tok-10",
        f"Host={reqs[0]['headers'].get('Host')} Path={reqs[0]['path']} token={reqs[0]['json'].get('token')}",
    )

    # Service 11: NAUTILIS Tapboard on-receipt
    r = call_test_forward(
        {"target": target, "mode": "receipt", "service": 11, "TOKEN": "tap-tok-11"},
        sample_1,
    )
    reqs = wait_http()
    check(
        "11. NAUTILIS Tapboard On-Receipt",
        r["ok"]
        and len(reqs) == 1
        and reqs[0]["headers"].get("Host") == "tapboard.nautilis.eu"
        and reqs[0]["headers"].get("User-Agent") == "Nautilis-iRelay"
        and reqs[0]["path"] == "/receive.php?token=tap-tok-11"
        and reqs[0]["json"].get("name") == "1-PaleAle",
        f"Host={reqs[0]['headers'].get('Host')} UA={reqs[0]['headers'].get('User-Agent')} Path={reqs[0]['path']}",
    )

    # ---- PART 2: PERIODIC RELAY SENSOR UPLOADERS (Services 0, 1, 2, 4, 9, 10, 11) ----

    # Periodic 0: UbiDots Nautilis_Thermometer
    r = call_test_forward(
        {
            "target": target,
            "mode": "periodic",
            "service": 0,
            "uri_custom": "BBFF-thermo-tok",
            "temp_yesno": 2,
            "temp_unit": 1,
            "relay_temp": 21.4,
        }
    )
    reqs = wait_http()
    check(
        "Periodic 0. UbiDots (Nautilis_Thermometer)",
        r["ok"]
        and len(reqs) == 1
        and reqs[0]["headers"].get("Host") == "industrial.api.ubidots.com"
        and reqs[0]["headers"].get("User-Agent") == "ESP8266"
        and reqs[0]["path"] == "/api/v1.6/devices/Nautilis_Thermometer?token=BBFF-thermo-tok"
        and abs(reqs[0]["json"]["temperature"] - 21.4) < 0.05,
        f"Path={reqs[0]['path']} body={reqs[0]['raw_body']}",
    )

    # Periodic 1: Brewfather Custom Stream
    r = call_test_forward(
        {
            "target": target,
            "mode": "periodic",
            "service": 1,
            "ap_ssid": "my-irelay",
            "uri_custom": "/stream?id=bfcustom1",
            "temp_yesno": 2,
            "temp_unit": 1,
            "pressure_unit": 1,
            "relay_temp": 19.8,
            "relay_bar": 1.15,
        }
    )
    reqs = wait_http()
    check(
        "Periodic 1. Brewfather Custom Stream",
        r["ok"]
        and len(reqs) == 1
        and reqs[0]["headers"].get("Host") == "log.brewfather.net"
        and reqs[0]["path"] == "/stream?id=bfcustom1"
        and reqs[0]["json"]["name"] == "my-irelay"
        and abs(reqs[0]["json"]["ext_temp"] - 19.8) < 0.05
        and reqs[0]["json"]["temp_unit"] == "C"
        and abs(reqs[0]["json"]["pressure"] - 1.15) < 0.05
        and reqs[0]["json"]["pressure_unit"] == "BAR",
        f"Path={reqs[0]['path']} body={reqs[0]['raw_body']}",
    )

    # Periodic 2: BrewSpy /api/nautilis
    r = call_test_forward(
        {
            "target": target,
            "mode": "periodic",
            "service": 2,
            "uri_custom": "bspy-relay-tok",
            "temp_yesno": 2,
            "temp_unit": 2,
            "pressure_unit": 2,
            "relay_temp": 20.0,
            "relay_psi": 14.7,
        }
    )
    reqs = wait_http()
    check(
        "Periodic 2. BrewSpy (/api/nautilis)",
        r["ok"]
        and len(reqs) == 1
        and reqs[0]["headers"].get("Host") == "brew-spy.com"
        and reqs[0]["path"] == "/api/nautilis"
        and reqs[0]["json"]["token"] == "bspy-relay-tok"
        and abs(reqs[0]["json"]["temperature"] - 68.0) < 0.05
        and reqs[0]["json"]["unit"] == "fahrenheit"
        and abs(reqs[0]["json"]["pressure"] - 14.7) < 0.05
        and reqs[0]["json"]["pressureUnit"] == "psi",
        f"Path={reqs[0]['path']} body={reqs[0]['raw_body']}",
    )

    # Periodic 4: Grainfather Custom Stream
    r = call_test_forward(
        {
            "target": target,
            "mode": "periodic",
            "service": 4,
            "uri_custom": "/iot/gf-custom/stream",
            "temp_yesno": 2,
            "temp_unit": 1,
            "relay_temp": 17.5,
        }
    )
    reqs = wait_http()
    check(
        "Periodic 4. Grainfather Custom Stream",
        r["ok"]
        and len(reqs) == 1
        and reqs[0]["headers"].get("Host") == "community.grainfather.com"
        and reqs[0]["path"] == "/iot/gf-custom/stream"
        and reqs[0]["json"]["specific_gravity"] == 0
        and abs(reqs[0]["json"]["temperature"] - 17.5) < 0.05
        and reqs[0]["json"]["unit"] == "celsius",
        f"Path={reqs[0]['path']} body={reqs[0]['raw_body']}",
    )

    # Periodic 9: MQTT Relay Sensor Publish
    r = call_test_forward(
        {
            "mode": "periodic",
            "service": 9,
            "uri": f"{HOST_IP}:{MQTT_PORT}",
            "ap_ssid": "nautilis-test",
            "temp_yesno": 2,
            "temp_unit": 1,
            "pressure_unit": 2,
            "relay_temp": 16.4,
            "relay_psi": 12.3,
        }
    )
    time.sleep(0.3)
    with mqtt_cond:
        pubs_p = list(captured_mqtt["publishes"])
    smap_p = {p["topic"]: p["payload"] for p in pubs_p if not p["topic"].startswith("homeassistant/")}
    check(
        "Periodic 9. MQTT Relay States",
        r["ok"]
        and "nautilis-test/relayrssi" in smap_p
        and "nautilis-test/rssi" in smap_p
        and smap_p.get("nautilis-test/temp") == "16.40"
        and smap_p.get("nautilis-test/pressure") == "12.30",
        f"relayrssi={smap_p.get('nautilis-test/relayrssi')} temp={smap_p.get('nautilis-test/temp')} pressure={smap_p.get('nautilis-test/pressure')}",
    )

    # Periodic 10: Generic HTTP Relay Sensor Upload
    r = call_test_forward(
        {
            "mode": "periodic",
            "service": 10,
            "server_address": target,
            "uri_custom": "/custom/relay",
            "temp_yesno": 2,
            "temp_unit": 1,
            "pressure_unit": 1,
            "relay_temp": 18.9,
            "relay_bar": 0.95,
        }
    )
    reqs = wait_http()
    check(
        "Periodic 10. Generic HTTP Relay Sensor",
        r["ok"]
        and len(reqs) == 1
        and reqs[0]["headers"].get("Host") == HOST_IP
        and reqs[0]["path"] == "/custom/relay"
        and abs(reqs[0]["json"]["ext_temp"] - 18.9) < 0.05
        and reqs[0]["json"]["temp_unit"] == "C"
        and abs(reqs[0]["json"]["pressure"] - 0.95) < 0.05
        and reqs[0]["json"]["pressure_unit"] == "BAR",
        f"Path={reqs[0]['path']} body={reqs[0]['raw_body']}",
    )

    # Periodic 11: NAUTILIS Tapboard Relay Sensor Upload
    r = call_test_forward(
        {
            "target": target,
            "mode": "periodic",
            "service": 11,
            "TOKEN": "tap-periodic-11",
            "temp_yesno": 2,
            "temp_unit": 1,
            "pressure_unit": 1,
            "relay_temp": 20.34,
            "relay_bar": 1.42,
        }
    )
    reqs = wait_http()
    check(
        "Periodic 11. NAUTILIS Tapboard Relay Sensor",
        r["ok"]
        and len(reqs) == 1
        and reqs[0]["headers"].get("Host") == "tapboard.nautilis.eu"
        and reqs[0]["headers"].get("User-Agent") == "Nautilis-iRelay"
        and reqs[0]["path"] == "/receive.php?token=tap-periodic-11"
        and abs(reqs[0]["json"]["temp"] - 20.3) < 0.05
        and reqs[0]["json"]["temp_unit"] == "C"
        and "alarm_temp1_low" in reqs[0]["json"]
        and "alarm_temp1_high" in reqs[0]["json"]
        and abs(reqs[0]["json"]["pressure"] - 1.42) < 0.05
        and reqs[0]["json"]["pressure_unit"] == "BAR"
        and "alarm_low" in reqs[0]["json"]
        and "alarm_high" in reqs[0]["json"],
        f"Path={reqs[0]['path']} body={reqs[0]['raw_body']}",
    )

    print(f"\nALL {len(results)}/{len(results)} LIVE HARDWARE FORWARDER MOCK TESTS PASSED!")
