#!/usr/bin/env python3
"""
Display WiFi connection status and IP address on OLED for 30 seconds.
No startup delay, exits automatically after 30 seconds of showing the IP.
"""

from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from PIL import ImageFont
import socket
import time
import os
import sys

def check_connection():
    """Check if connected to the internet"""
    try:
        # Try to connect to Google's DNS
        socket.create_connection(("8.8.8.8", 53), timeout=3)
        return True
    except OSError:
        return False

def get_ip_address():
    """Get the IP address of the Raspberry Pi"""
    try:
        # Try getting wireless IP
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        try:
            # Try different interfaces if the above fails
            for interface in ['wlan0', 'eth0', 'usb0']:
                ip = os.popen(f'ip addr show {interface} | grep "\<inet\>" | awk \'{{print $2}}\' | awk -F "/" \'{{print $1}}\'').read().strip()
                if ip and ip != "127.0.0.1":
                    return ip
        except:
            pass
        
        return "No IP found"

def main():
    # Setup logging
    log_file = '/home/bumpy1/ip_display.log'
    
    try:
        # Initialize the I2C interface
        serial = i2c(port=1, address=0x3C)  # Change address if needed
        
        # Initialize the OLED display
        device = ssd1306(serial, width=128, height=64)
        
        # Try to load a font, fallback to default if not available
        try:
            font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14)
            small_font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 10)
        except IOError:
            font = ImageFont.load_default()
            small_font = ImageFont.load_default()
        
        # Get hostname
        hostname = socket.gethostname()
        
        # Wait for internet connection - with animation
        dots = 0
        connection_start_time = time.time()
        
        # Show "Connecting" until connected or timeout (30 seconds)
        while not check_connection():
            with canvas(device) as draw:
                # Draw a border
                draw.rectangle(device.bounding_box, outline="white", fill="black")
                
                # Display the hostname
                draw.text((4, 2), f"Host: {hostname}", fill="white", font=font)
                
                # Display connecting message with animated dots
                connecting_text = "Connecting to WiFi" + "." * dots
                draw.text((4, 30), connecting_text, fill="white", font=font)
                
                # Show Karthikesh Robotics
                draw.text((4, 50), "Karthikesh Robotics", fill="white", font=small_font)
            
            # Update dots animation
            dots = (dots + 1) % 4
            
            # Check if we've waited 30 seconds for connection
            if time.time() - connection_start_time > 30:
                # Show "No Connection" for 5 seconds then exit
                for i in range(5):
                    with canvas(device) as draw:
                        draw.rectangle(device.bounding_box, outline="white", fill="black")
                        draw.text((4, 2), f"Host: {hostname}", fill="white", font=font)
                        draw.text((4, 30), "No WiFi Connection", fill="white", font=font)
                        draw.text((4, 50), "Karthikesh Robotics", fill="white", font=small_font)
                    time.sleep(1)
                device.clear()
                return
            
            time.sleep(0.5)
        
        # Get the IP address once connected
        ip_address = get_ip_address()
        with open(log_file, 'w') as f:
            f.write(f"Connected! IP Address: {ip_address}\n")
        
        # Display IP for exactly 30 seconds
        display_start_time = time.time()
        
        while time.time() - display_start_time < 30:
            with canvas(device) as draw:
                # Draw a border
                draw.rectangle(device.bounding_box, outline="white", fill="black")
                
                # Display the hostname
                draw.text((4, 2), f"Host: {hostname}", fill="white", font=font)
                
                # Display IP address info
                draw.text((4, 22), "IP Address:", fill="white", font=small_font)
                draw.text((4, 37), f"{ip_address}", fill="white", font=font)
                
                # Show Karthikesh Robotics instead of countdown
                draw.text((4, 52), "Karthikesh Robotics", fill="white", font=small_font)
            
            # Update every 1 second
            time.sleep(1)
    
    except Exception as e:
        # Log any errors
        with open(log_file, 'w') as f:
            f.write(f"Error: {str(e)}\n")
    
    finally:
        # Clear the display before exiting
        try:
            device.clear()
        except:
            pass

if __name__ == "__main__":
    main()
