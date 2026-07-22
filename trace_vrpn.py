#!/usr/bin/env python3
import socket
import struct
import time
import subprocess
import platform
import json
import os
import sys

def load_config():
    """Attempts to load target host and port from config.json."""
    host = "192.168.1.100"
    port = 3883
    if os.path.exists("config.json"):
        try:
            with open("config.json", "r") as f:
                config = json.load(f)
                vrpn_config = config.get("vrpn", {})
                host = vrpn_config.get("host", host)
                port = vrpn_config.get("port", port)
                print(f"[INFO] Loaded targets from config.json: {host}:{port}")
        except Exception as e:
            print(f"[WARNING] Failed to parse config.json: {e}. Using defaults.")
    else:
        print(f"[INFO] config.json not found. Using defaults: {host}:{port}")
    return host, port

def print_section(title):
    print("\n" + "=" * 60)
    print(f" {title} ")
    print("=" * 60)

def check_local_interfaces(target_host):
    print_section("1. LOCAL NETWORK INTERFACES")
    
    # Try to find default local IP by connecting to target
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Doesn't actually establish a connection, just gets routing path
        s.connect((target_host, 80))
        local_ip = s.getsockname()[0]
        s.close()
        print(f"Default route to {target_host} is via local IP: {local_ip}")
    except Exception as e:
        print(f"Could not determine default local IP to {target_host}: {e}")
        local_ip = None

    # Retrieve all IPs associated with hostname
    try:
        hostname = socket.gethostname()
        print(f"Hostname: {hostname}")
        ips = socket.gethostbyname_ex(hostname)[2]
        print(f"All resolved local IPs: {ips}")
    except Exception as e:
        print(f"Could not list host IPs: {e}")

    # Run ifconfig (Mac/Linux) or ipconfig (Windows) for adapter details
    system_name = platform.system()
    if system_name in ["Darwin", "Linux"]:
        try:
            res = subprocess.run(["ifconfig"], capture_output=True, text=True)
            print("\nActive interfaces with IPs:")
            for line in res.stdout.split("\n"):
                if "inet " in line or "status: active" in line or "flags=" in line:
                    # Filter relevant adapter lines to keep output clean
                    if not line.strip().startswith("inet6"):
                        print(f"  {line.strip()}")
        except Exception as e:
            print(f"Failed to run ifconfig: {e}")
    elif system_name == "Windows":
        try:
            res = subprocess.run(["ipconfig"], capture_output=True, text=True)
            for line in res.stdout.split("\n"):
                if "IPv4" in line or "Subnet Mask" in line or "Adapter" in line:
                    print(f"  {line.strip()}")
        except Exception as e:
            print(f"Failed to run ipconfig: {e}")

def run_ping(target_host):
    print_section(f"2. PING TEST ({target_host})")
    system_name = platform.system()
    cmd = ["ping", "-c", "3", target_host] if system_name != "Windows" else ["ping", "-n", "3", target_host]
    
    print(f"Executing: {' '.join(cmd)}")
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        print(res.stdout)
        if res.returncode == 0:
            print("[SUCCESS] Target host is pingable.")
        else:
            print("[ERROR] Ping failed. Host might be offline, firewall blocked, or unreachable.")
    except subprocess.TimeoutExpired:
        print("[TIMEOUT] Ping timed out.")
    except Exception as e:
        print(f"[ERROR] Failed to run ping: {e}")

def test_tcp_server_first(host, port):
    print_section("3. TCP HANDSHAKE TEST - SERVER FIRST (Standard VRPN)")
    print(f"Connecting to {host}:{port} via TCP...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5.0)
    
    try:
        start_time = time.time()
        s.connect((host, port))
        conn_time = time.time() - start_time
        print(f"[SUCCESS] Connected successfully in {conn_time:.3f} seconds.")
        print("Waiting for server to send the VRPN cookie (5s timeout)...")
        
        data = s.recv(24)
        if not data:
            print("[WARNING] Server closed connection immediately (returned 0 bytes/EOF).")
        else:
            print(f"[SUCCESS] Received {len(data)} bytes from server.")
            print(f"  Hex: {data.hex()}")
            try:
                ascii_data = data.split(b"\x00")[0].decode("utf-8", errors="replace")
                print(f"  ASCII: '{ascii_data}'")
            except Exception as e:
                print(f"  Could not decode ASCII: {e}")
                
    except socket.timeout:
        print("[TIMEOUT] Socket timed out waiting for server cookie. Server accepted connection but sent nothing.")
    except ConnectionRefusedError:
        print("[ERROR] Connection refused. Is the VRPN server actually running?")
    except Exception as e:
        print(f"[ERROR] TCP Connection/Read failed: {e}")
    finally:
        s.close()

def test_tcp_client_first(host, port):
    print_section("4. TCP HANDSHAKE TEST - CLIENT FIRST (Alternative)")
    print(f"Connecting to {host}:{port} via TCP...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5.0)
    
    # Common VRPN cookies (padded to 24 bytes)
    versions = ["vrpn: ver. 07.36  0", "vrpn: ver. 07.33  0", "vrpn: ver. 07.30  0"]
    
    try:
        s.connect((host, port))
        print("[SUCCESS] Connected successfully. Sending cookies client-first...")
        
        for version in versions:
            cookie = version.encode("utf-8")
            cookie = cookie + (b"\x00" * (24 - len(cookie)))
            print(f"Sending client version: '{version}' ({len(cookie)} bytes)")
            s.sendall(cookie)
            
            print("Waiting for server response (2.0s)...")
            s.settimeout(2.0)
            try:
                data = s.recv(24)
                if not data:
                    print("  Server closed connection.")
                    break
                else:
                    print(f"  [RECEIVED] {len(data)} bytes from server:")
                    print(f"    Hex: {data.hex()}")
                    ascii_data = data.split(b"\x00")[0].decode("utf-8", errors="replace")
                    print(f"    ASCII: '{ascii_data}'")
                    return
            except socket.timeout:
                print("  No response yet. Trying next version if available...")
                
    except ConnectionRefusedError:
        print("[ERROR] Connection refused.")
    except Exception as e:
        print(f"[ERROR] client-first test error: {e}")
    finally:
        s.close()

def listen_udp_broadcast(port):
    print_section(f"5. UDP PACKET SNIFFER (Port {port})")
    print(f"Binding UDP socket to 0.0.0.0:{port}...")
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(5.0)
    
    try:
        s.bind(("0.0.0.0", port))
        print("Listening for UDP unicast/broadcast packets (5s)...")
        start = time.time()
        while time.time() - start < 5.0:
            try:
                data, addr = s.recvfrom(2048)
                print(f"\n[RECEIVED UDP] From {addr[0]}:{addr[1]} | Length: {len(data)} bytes")
                print(f"  Hex snippet: {data[:64].hex()}")
                # Check if it looks like VRPN header or text
                try:
                    text_snippet = data[:64].split(b"\x00")[0].decode("utf-8", errors="ignore")
                    if text_snippet.isprintable() and len(text_snippet) > 3:
                        print(f"  ASCII preview: '{text_snippet}'")
                except Exception:
                    pass
            except socket.timeout:
                pass
        print("\nFinished UDP listening window.")
    except Exception as e:
        print(f"[ERROR] Failed to listen on UDP: {e}")
    finally:
        s.close()

def main():
    host, port = load_config()
    
    # Allow command line overrides
    if len(sys.argv) > 1:
        host = sys.argv[1]
    if len(sys.argv) > 2:
        try:
            port = int(sys.argv[2])
        except ValueError:
            pass
            
    print(f"Running VRPN Diagnostics targeting {host}:{port}")
    
    check_local_interfaces(host)
    run_ping(host)
    test_tcp_server_first(host, port)
    test_tcp_client_first(host, port)
    listen_udp_broadcast(port)
    
    print_section("6. RECOMMENDATIONS / NEXT STEPS")
    print("If TCP standard connect timed out, but ping succeeded:")
    print(" 1. The VRPN server application might not be running or might be configured to use UDP only.")
    print(" 2. A local or remote firewall (e.g. Windows Firewall on the VRPN server) might be blocking port 3883 TCP.")
    print(" 3. Run packet capture using tcpdump on your macOS terminal to inspect live packets:")
    print("    Identify your LAN interface (e.g. en14) and run:")
    print("    sudo tcpdump -i en14 -nXX port 3883")
    print("\nIf UDP packets were captured:")
    print(" - It indicates the server is streaming data via UDP broadcasts directly instead of utilizing a TCP handshake.")
    print(" - Look closely at the VRPN server settings in Motive (e.g., Broadcast/Multicast vs. TCP/Unicast).")

if __name__ == "__main__":
    main()
