"""
TCP server for receiving GNSS data from devices.
Forwards data to the main application via ZMQ.

Expected data format from GNSS devices:
{
    "client_id": 0,  # 0, 1, 2 for anchors
    "lat": 22.31930,
    "lng": 114.16940,
    "alt": 10.0  # optional
}
"""

from __future__ import annotations

import socket
import json
import threading
from typing import Optional

try:
    import zmq
    HAS_ZMQ = True
except ImportError:
    HAS_ZMQ = False
    print("Warning: pyzmq not installed. Install with: pip install pyzmq")


# Configuration
ZMQ_BIND_ADDR = "tcp://127.0.0.1:5555"  # ZMQ PUSH endpoint
TCP_HOST = "0.0.0.0"
TCP_PORT = 7799


class TCPServer:
    """TCP server for GNSS data reception."""
    
    def __init__(self, host: str = TCP_HOST, port: int = TCP_PORT, zmq_addr: str = ZMQ_BIND_ADDR):
        self.host = host
        self.port = port
        self.zmq_addr = zmq_addr
        self._zmq_context: Optional[zmq.Context] = None
        self._zmq_socket: Optional[zmq.Socket] = None
        self._server_socket: Optional[socket.socket] = None
        self._stop_event = threading.Event()
    
    def start(self):
        """Start the TCP server."""
        if not HAS_ZMQ:
            print("[TCP Server] ZMQ not available, running without forwarding")
            self._zmq_socket = None
        else:
            # Initialize ZMQ
            self._zmq_context = zmq.Context()
            self._zmq_socket = self._zmq_context.socket(zmq.PUSH)
            self._zmq_socket.bind(self.zmq_addr)
            print(f"[TCP Server] ZMQ PUSH bound to {self.zmq_addr}")
        
        # Start TCP server
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_socket.bind((self.host, self.port))
        self._server_socket.listen(5)
        print(f"[TCP Server] Listening on {self.host}:{self.port}")
        
        # Accept connections
        self._stop_event.clear()
        while not self._stop_event.is_set():
            try:
                self._server_socket.settimeout(1.0)
                conn, addr = self._server_socket.accept()
                threading.Thread(
                    target=self._handle_client,
                    args=(conn, addr),
                    daemon=True
                ).start()
            except socket.timeout:
                continue
            except Exception as e:
                if not self._stop_event.is_set():
                    print(f"[TCP Server] Accept error: {e}")
    
    def _handle_client(self, conn: socket.socket, addr: tuple):
        """Handle a connected client."""
        print(f"[TCP] Connected by {addr}")
        buffer = b''
        
        try:
            while not self._stop_event.is_set():
                try:
                    conn.settimeout(1.0)
                    data = conn.recv(1024)
                    if not data:
                        break
                    buffer += data
                    
                    # Parse JSON messages (newline delimited)
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        if not line.strip():
                            continue
                        
                        try:
                            msg = json.loads(line.decode().strip())
                            
                            # Validate required fields
                            if all(k in msg for k in ['client_id', 'lat', 'lng']):
                                print(f"[TCP] Valid data: client_id={msg['client_id']}, "
                                      f"lat={msg['lat']:.6f}, lng={msg['lng']:.6f}")
                                
                                # Forward via ZMQ
                                if self._zmq_socket:
                                    self._zmq_socket.send_json(msg)
                            else:
                                print(f"[TCP] Invalid data (missing fields): {msg}")
                        except json.JSONDecodeError as e:
                            print(f"[TCP] JSON parse error: {e}")
                except socket.timeout:
                    continue
        except Exception as e:
            print(f"[TCP] Client error: {e}")
        finally:
            conn.close()
            print(f"[TCP] Disconnected {addr}")
    
    def stop(self):
        """Stop the TCP server."""
        self._stop_event.set()
        if self._server_socket:
            try:
                self._server_socket.close()
            except:
                pass
        if self._zmq_socket:
            try:
                self._zmq_socket.close()
            except:
                pass
        if self._zmq_context:
            try:
                self._zmq_context.term()
            except:
                pass
        print("[TCP Server] Stopped")


def main():
    """Main entry point."""
    server = TCPServer()
    try:
        server.start()
    except KeyboardInterrupt:
        print("\n[TCP Server] Interrupted")
    finally:
        server.stop()


if __name__ == '__main__':
    main()