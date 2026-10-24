#!/usr/bin/env python3
"""Launch the local HTTP server for the NIDAR RescueSwarm GCS Dashboard."""

from http.server import SimpleHTTPRequestHandler
import socketserver
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
GCS_DIR = ROOT / "gcs"
PORT = 8080


class GCSHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(GCS_DIR), **kwargs)

    def log_message(self, format, *args):
        # Concise logging
        sys.stderr.write(f"[GCS Server] {self.address_string()} - {format % args}\n")


def main() -> None:
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", PORT), GCSHandler) as httpd:
        print("=" * 65)
        print("  NIDAR RescueSwarm — Ground Control Station (GCS) Dashboard")
        print(f"  Live URL: http://localhost:{PORT}")
        print("=" * 65)
        print("Press Ctrl+C to terminate.")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nShutting down GCS server.")


if __name__ == "__main__":
    main()
