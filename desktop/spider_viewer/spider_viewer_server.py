#!/usr/bin/env python3
import argparse
import json
import os
import queue
import socket
import threading
import time
import urllib.parse
import webbrowser
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer


CLIENTS = []
CLIENTS_LOCK = threading.Lock()


def parse_packet(packet):
    fields = packet.strip().split()
    if len(fields) != 7 or fields[0] != "SPIDER_SENSOR_V1":
        return None
    return {
        "sequence_id": int(fields[1]),
        "timestamp_ms": int(fields[2]),
        "name": fields[3],
        "value": float(fields[4]),
        "unit": "" if fields[5] == "-" else fields[5],
        "status": fields[6],
        "received_ms": int(time.time() * 1000),
    }


def udp_listener(host, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    while True:
        data, _addr = sock.recvfrom(2048)
        try:
            sample = parse_packet(data.decode("utf-8", errors="replace"))
        except ValueError:
            sample = None
        if sample is None:
            continue
        payload = json.dumps(sample, separators=(",", ":"))
        with CLIENTS_LOCK:
            stale_clients = []
            for client in CLIENTS:
                try:
                    client.put_nowait(payload)
                except queue.Full:
                    stale_clients.append(client)
            for client in stale_clients:
                CLIENTS.remove(client)


class ViewerHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, directory=None, video_url="", **kwargs):
        self.video_url = video_url
        super().__init__(*args, directory=directory, **kwargs)

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            client = queue.Queue(maxsize=200)
            with CLIENTS_LOCK:
                CLIENTS.append(client)
            try:
                while True:
                    payload = client.get(timeout=15.0)
                    self.wfile.write(("data: " + payload + "\n\n").encode("utf-8"))
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, queue.Empty):
                pass
            finally:
                with CLIENTS_LOCK:
                    if client in CLIENTS:
                        CLIENTS.remove(client)
            return
        if parsed.path == "/config.json":
            body = json.dumps({"video_url": self.video_url}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        return super().do_GET()


def make_handler(directory, video_url):
    class BoundViewerHandler(ViewerHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=directory, video_url=video_url, **kwargs)

    return BoundViewerHandler


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--udp-bind", default="0.0.0.0")
    parser.add_argument("--udp-port", type=int, default=45455)
    parser.add_argument("--http-bind", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8088)
    parser.add_argument("--video-url", default="")
    parser.add_argument("--open", action="store_true")
    args = parser.parse_args()

    app_dir = os.path.dirname(os.path.abspath(__file__))
    threading.Thread(target=udp_listener, args=(args.udp_bind, args.udp_port), daemon=True).start()
    server = ThreadingHTTPServer((args.http_bind, args.http_port), make_handler(app_dir, args.video_url))
    url = f"http://{args.http_bind}:{args.http_port}/index.html"
    print(f"Spider viewer: {url}")
    print(f"Receiving telemetry UDP on {args.udp_bind}:{args.udp_port}")
    if args.open:
        webbrowser.open(url)
    server.serve_forever()


if __name__ == "__main__":
    main()
