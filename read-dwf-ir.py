#!/usr/bin/env python3
# /// script
# requires-python = ">=3.8"
# dependencies = [
#     "rich",
# ]
# ///
"""
read-dwf-ir.py

Interfaces with Digilent WaveForms devices (e.g., Analog Discovery 3) to capture and
decode infrared remote signals. It automatically handles double packets with pauses,
identifies bit sequences, calculates thresholds, and generates ESPHome configurations.
"""

import sys
import ctypes
import time
import argparse
import json
import os

from rich.console import Console
from rich.table import Table

console = Console()

# Digilent WaveForms SDK State Constants
DwfStateDone = 2
DwfStateArmed = 1
trigsrcDetectorDigitalIn = 3

def load_dwf_library():
    """Load the DWF library using ctypes, handling multiple platforms."""
    if sys.platform.startswith("win"):
        try:
            return ctypes.cdll.dwf
        except Exception:
            console.print("[bold red]Error:[/bold red] Could not load dwf.dll. Ensure Digilent WaveForms is installed and added to PATH.")
            sys.exit(1)
    elif sys.platform.startswith("darwin"):
        paths = [
            "/Library/Frameworks/dwf.framework/dwf",
            "/Applications/WaveForms.app/Contents/Frameworks/dwf.framework/dwf",
            "/Applications/WaveForms.app/Contents/Resources/SDK/libdwf.dylib",
            "libdwf.dylib"
        ]
        for path in paths:
            try:
                return ctypes.CDLL(path)
            except OSError:
                continue
        console.print("[bold red]Error:[/bold red] Could not load libdwf on macOS. Checked paths:")
        for p in paths:
            console.print(f"  - {p}")
        sys.exit(1)
    else:
        # Linux
        paths = [
            "libdwf.so",
            "/usr/lib/libdwf.so",
            "/usr/local/lib/libdwf.so"
        ]
        for path in paths:
            try:
                return ctypes.CDLL(path)
            except OSError:
                continue
        console.print("[bold red]Error:[/bold red] Could not load libdwf.so on Linux.")
        sys.exit(1)

def setup_device(dwf, pin, sample_rate, duration, enable_power):
    """Open and configure the Digilent device logic analyzer."""
    # Declare enumeration and error function signatures
    dwf.FDwfEnum.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
    dwf.FDwfEnum.restype = ctypes.c_int
    dwf.FDwfEnumDeviceName.argtypes = [ctypes.c_int, ctypes.c_char_p]
    dwf.FDwfEnumDeviceName.restype = ctypes.c_int
    dwf.FDwfEnumSN.argtypes = [ctypes.c_int, ctypes.c_char_p]
    dwf.FDwfEnumSN.restype = ctypes.c_int
    dwf.FDwfGetLastErrorMsg.argtypes = [ctypes.c_char_p]
    dwf.FDwfGetLastErrorMsg.restype = ctypes.c_int
    
    # Declare AnalogIO and DigitalIO signatures
    dwf.FDwfAnalogIOChannelNodeSet.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_double]
    dwf.FDwfAnalogIOChannelNodeSet.restype = ctypes.c_int
    dwf.FDwfAnalogIOEnableSet.argtypes = [ctypes.c_int, ctypes.c_int]
    dwf.FDwfAnalogIOEnableSet.restype = ctypes.c_int
    dwf.FDwfDigitalIOStatus.argtypes = [ctypes.c_int]
    dwf.FDwfDigitalIOStatus.restype = ctypes.c_int
    dwf.FDwfDigitalIOInputStatus.argtypes = [ctypes.c_int, ctypes.POINTER(ctypes.c_uint32)]
    dwf.FDwfDigitalIOInputStatus.restype = ctypes.c_int

    # Scan for devices
    c_devices = ctypes.c_int()
    console.print("[yellow]Scanning for connected Digilent devices...[/yellow]")
    dwf.FDwfEnum(ctypes.c_int(0), ctypes.byref(c_devices))
    console.print(f"[bold blue]Devices found:[/bold blue] {c_devices.value}")
    
    if c_devices.value == 0:
        console.print("[bold red]Error: No Digilent devices detected.[/bold red]")
        console.print("[yellow]Troubleshooting tips:[/yellow]")
        console.print("  1. Make sure your device (e.g. Analog Discovery 3) is connected to a USB port.")
        console.print("  2. If using a USB hub, try plugging the device directly into the Mac.")
        console.print("  3. Unplug and replug the USB cable to force re-enumeration.")
        console.print("  4. Make sure the Digilent Adept Runtime is running and matches your OS.")
        sys.exit(1)
        
    for i in range(c_devices.value):
        devicename = ctypes.create_string_buffer(64)
        serialnum = ctypes.create_string_buffer(16)
        dwf.FDwfEnumDeviceName(ctypes.c_int(i), devicename)
        dwf.FDwfEnumSN(ctypes.c_int(i), serialnum)
        console.print(f"  Device {i}: [cyan]{devicename.value.decode('utf-8')}[/cyan] (S/N: {serialnum.value.decode('utf-8')})")

    # Connect to device 0
    hdwf = ctypes.c_int()
    console.print("[yellow]Connecting to device 0...[/yellow]")
    dwf.FDwfDeviceOpen(ctypes.c_int(0), ctypes.byref(hdwf))
    
    if hdwf.value == 0:
        szerr = ctypes.create_string_buffer(512)
        dwf.FDwfGetLastErrorMsg(szerr)
        error_msg = szerr.value.decode('utf-8', errors='ignore').strip()
        
        console.print(f"[bold red]Error: Could not open Digilent device.[/bold red]")
        console.print(f"[bold red]Reason from SDK:[/bold red] {error_msg}")
        console.print("[yellow]Troubleshooting tips:[/yellow]")
        console.print("  1. Completely close the WaveForms GUI application. Only one application can control the hardware.")
        console.print("  2. If another python script crashed, it may have left the device connection open. Unplug and replug the USB cable to reset it.")
        sys.exit(1)
        
    # Enable V+ supply if requested
    if enable_power:
        console.print("[yellow]Enabling positive user power supply (V+) at 3.3V...[/yellow]")
        # Channel 0 (positive supply), Node 0 (enable = 1.0)
        dwf.FDwfAnalogIOChannelNodeSet(hdwf, ctypes.c_int(0), ctypes.c_int(0), ctypes.c_double(1.0))
        # Channel 0 (positive supply), Node 1 (voltage = 3.3V)
        dwf.FDwfAnalogIOChannelNodeSet(hdwf, ctypes.c_int(0), ctypes.c_int(1), ctypes.c_double(3.3))
        # Master Enable
        dwf.FDwfAnalogIOEnableSet(hdwf, ctypes.c_int(1))
        time.sleep(0.2) # wait for voltage to settle
        
    # Read live status of the target pin
    dwf.FDwfDigitalIOStatus(hdwf)
    pins_state = ctypes.c_uint32()
    dwf.FDwfDigitalIOInputStatus(hdwf, ctypes.byref(pins_state))
    initial_state = (pins_state.value >> pin) & 1
    console.print(f"[bold blue]Live pin status (DIO {pin}):[/bold blue] {'HIGH (1)' if initial_state else 'LOW (0)'}")
    
    if initial_state == 0:
        console.print("[bold yellow]Warning: DIO pin is currently LOW.[/bold yellow]")
        console.print("  Standard active-low IR receivers (like TSOP38238) must idle [bold green]HIGH[/bold green] when no signal is present.")
        console.print("  If it is LOW, check:")
        console.print("    1. Is the IR receiver powered? (Use `--power` if VCC is connected to V+)")
        console.print("    2. Are the VOUT, GND, and VCC pins connected correctly? (reversed pins will short and stay LOW)")
        console.print("    3. Do you need a pull-up resistor from VOUT to VCC? (usually 10k ohm if not built-in)")
        
    # Get internal clock info
    hzFreq = ctypes.c_double()
    dwf.FDwfDigitalInInternalClockInfo(hdwf, ctypes.byref(hzFreq))
    console.print(f"[bold blue]Device internal clock:[/bold blue] {hzFreq.value / 1e6:.2f} MHz")
    
    # Calculate divider
    divider = int(hzFreq.value / sample_rate)
    actual_sample_rate = hzFreq.value / divider
    console.print(f"[bold blue]Configuring sample rate:[/bold blue] {actual_sample_rate / 1000.0:.2f} kHz (divider={divider})")
    dwf.FDwfDigitalInDividerSet(hdwf, ctypes.c_int(divider))
    
    # Configure 16-bit format (captures pins 0-15)
    dwf.FDwfDigitalInSampleFormatSet(hdwf, ctypes.c_int(16))
    
    # Get max buffer size
    max_samples = ctypes.c_int()
    dwf.FDwfDigitalInBufferSizeInfo(hdwf, ctypes.byref(max_samples))
    console.print(f"[bold blue]Device maximum buffer size:[/bold blue] {max_samples.value} samples")
    
    # Calculate required samples for duration
    desired_samples = int(duration * actual_sample_rate)
    num_samples = min(desired_samples, max_samples.value)
    
    # If the max buffer size cannot hold the desired duration, lower the sample rate
    if num_samples < desired_samples:
        console.print(f"[bold yellow]Warning:[/bold yellow] Buffer size {max_samples.value} is too small for {duration:.2f}s at {actual_sample_rate/1000.0:.2f} kHz.")
        actual_sample_rate = max_samples.value / duration
        divider = int(hzFreq.value / actual_sample_rate)
        dwf.FDwfDigitalInDividerSet(hdwf, ctypes.c_int(divider))
        actual_sample_rate = hzFreq.value / divider
        num_samples = max_samples.value
        console.print(f"[bold blue]Adjusted sample rate:[/bold blue] {actual_sample_rate/1000.0:.2f} kHz to fit {duration:.2f}s duration.")
        
    dwf.FDwfDigitalInBufferSizeSet(hdwf, ctypes.c_int(num_samples))
    console.print(f"[bold blue]Buffer size configured:[/bold blue] {num_samples} samples ({num_samples / actual_sample_rate * 1000.0:.1f} ms capture window)")
    
    # Set trigger position: capture 5% pre-trigger and 95% post-trigger
    trigger_pos = int(num_samples * 0.05)
    dwf.FDwfDigitalInTriggerPositionSet(hdwf, ctypes.c_int(num_samples - trigger_pos))
    
    # Set trigger source to digital detector
    dwf.FDwfDigitalInTriggerSourceSet(hdwf, ctypes.c_byte(trigsrcDetectorDigitalIn))
    
    # Set trigger on falling edge of the selected DIO pin
    # (level low, level high, edge rise, edge fall)
    dwf.FDwfDigitalInTriggerSet(hdwf, ctypes.c_uint(0), ctypes.c_uint(0), ctypes.c_uint(0), ctypes.c_uint(1 << pin))
    console.print(f"[bold green]Trigger armed on falling edge of DIO pin {pin}.[/bold green]")
    
    return hdwf, actual_sample_rate, num_samples

def acquire_data(dwf, hdwf, num_samples, button_name):
    """Arm the Logic Analyzer and wait for trigger completion."""
    # Configure and start DigitalIn (fStart=False, fConfigure=True to arm)
    dwf.FDwfDigitalInConfigure(hdwf, ctypes.c_bool(False), ctypes.c_bool(True))
    
    console.print(f"\n[bold green][Ready][/bold green] Press button [bold cyan]'{button_name}'[/bold cyan] now (or press Ctrl+C to cancel)...")
    
    sts = ctypes.c_byte()
    try:
        while True:
            # Poll status (fReadData=True)
            dwf.FDwfDigitalInStatus(hdwf, ctypes.c_bool(True), ctypes.byref(sts))
            if sts.value == DwfStateDone:
                break
            time.sleep(0.01)
    except KeyboardInterrupt:
        console.print("\n[bold red]Acquisition cancelled.[/bold red]")
        # Disarm device
        dwf.FDwfDigitalInConfigure(hdwf, ctypes.c_bool(False), ctypes.c_bool(False))
        return "CANCEL"
        
    console.print("[green]Data captured! Transferring from device...[/green]")
    # Create buffer and retrieve data
    rgwSamples = (ctypes.c_uint16 * num_samples)()
    dwf.FDwfDigitalInStatusData(hdwf, rgwSamples, ctypes.c_int(num_samples * 2))
    
    return list(rgwSamples)

def process_samples(samples, pin, actual_sample_rate, evaluate_field="space"):
    """Parse raw sample bits into edge transitions, packet segments, and bit streams."""
    num_samples = len(samples)
    pin_samples = [(samples[i] >> pin) & 1 for i in range(num_samples)]
    
    # Find transitions (alternating states and their durations in us)
    pulses = []
    current_state = pin_samples[0]
    count = 1
    
    for i in range(1, num_samples):
        if pin_samples[i] == current_state:
            count += 1
        else:
            duration_us = (count / actual_sample_rate) * 1_000_000.0
            pulses.append((current_state, duration_us))
            current_state = pin_samples[i]
            count = 1
    # Last pulse
    duration_us = (count / actual_sample_rate) * 1_000_000.0
    pulses.append((current_state, duration_us))
    
    # Find the first transition to 0 (LOW) which is the start of the transmission
    start_idx = -1
    for idx, (state, duration) in enumerate(pulses):
        if state == 0:
            start_idx = idx
            break
            
    if start_idx == -1:
        console.print("[bold red]Error:[/bold red] No falling edge (carrier active) detected in the signal.")
        return None
        
    transmission = pulses[start_idx:]
    
    # Split transmission into packets based on gaps longer than 10ms (10,000us)
    packets = []
    current_packet = []
    for state, duration in transmission:
        if state == 1 and duration > 10000.0:
            if current_packet:
                packets.append(current_packet)
                current_packet = []
        else:
            current_packet.append((state, duration))
    if current_packet:
        packets.append(current_packet)
        
    console.print(f"[bold blue]Detected packets:[/bold blue] {len(packets)}")
    
    # Gather target bit intervals across all packets to calculate threshold
    target_state = 1 if evaluate_field == "space" else 0
    all_intervals = []
    for p in packets:
        for state, dur in p:
            if state == target_state and 200.0 <= dur <= 3000.0:
                all_intervals.append(dur)
                
    # Differentiate between short and long pulses/spaces
    threshold = 1000.0
    if len(all_intervals) >= 4:
        min_val = min(all_intervals)
        max_val = max(all_intervals)
        if max_val - min_val > 300.0:
            threshold = (min_val + max_val) / 2.0
            console.print(f"[bold blue]Calculated bit {evaluate_field} threshold:[/bold blue] {threshold:.1f} us (min={min_val:.1f}us, max={max_val:.1f}us)")
        else:
            console.print(f"[bold yellow]Uniform bit {evaluate_field}s detected.[/bold yellow] Using default threshold: {threshold:.1f} us")
    else:
        console.print(f"[bold yellow]Insufficient bit intervals to calculate dynamic threshold.[/bold yellow] Using default: {threshold:.1f} us")
        
    processed_packets = []
    for p_idx, p in enumerate(packets):
        # Identify leader mark and space (e.g., >3ms mark and >2ms space)
        leader_mark = None
        leader_space = None
        data_start = 0
        
        if len(p) >= 2 and p[0][0] == 0 and p[0][1] > 3000.0:
            leader_mark = p[0][1]
            if p[1][0] == 1 and p[1][1] > 2000.0:
                leader_space = p[1][1]
                data_start = 2
            else:
                data_start = 1
                
        bits = []
        raw_timings = []
        
        for state, dur in p:
            val = int(round(dur))
            # ESPHome raw style: positive for Mark (LOW), negative for Space (HIGH)
            if state == 0:
                raw_timings.append(val)
            else:
                raw_timings.append(-val)
                
        # Parse bits starting after leader
        for idx in range(data_start, len(p)):
            state, dur = p[idx]
            if state == target_state and 200.0 <= dur <= 4000.0:
                if dur < threshold:
                    bits.append('0')
                else:
                    bits.append('1')
                    
        bit_str = "".join(bits)
        processed_packets.append({
            "packet_index": p_idx,
            "leader_mark_us": leader_mark,
            "leader_space_us": leader_space,
            "bits": bit_str,
            "raw_timings": raw_timings
        })
        
    return {
        "threshold_us": threshold,
        "packets": processed_packets
    }

def to_hex_lsb(bit_str):
    """Convert a binary string to hex bytes assuming LSB-first transmission."""
    padded = bit_str + "0" * ((8 - len(bit_str) % 8) % 8)
    bytes_list = []
    for i in range(0, len(padded), 8):
        byte_bits = padded[i:i+8]
        # Reverse bit order for LSB first
        reversed_bits = byte_bits[::-1]
        bytes_list.append(f"0x{int(reversed_bits, 2):02X}")
    return ", ".join(bytes_list)

def to_hex_msb(bit_str):
    """Convert a binary string to hex bytes assuming MSB-first transmission."""
    padded = bit_str + "0" * ((8 - len(bit_str) % 8) % 8)
    bytes_list = []
    for i in range(0, len(padded), 8):
        byte_bits = padded[i:i+8]
        bytes_list.append(f"0x{int(byte_bits, 2):02X}")
    return ", ".join(bytes_list)

def main():
    parser = argparse.ArgumentParser(description="Digilent WaveForms IR Receiver Decoder")
    parser.add_argument("--pin", type=int, default=0, help="Digital input pin connected to the IR receiver (default: DIO 0)")
    parser.add_argument("--rate", type=float, default=100000.0, help="Sampling rate in Hz (default: 100000)")
    parser.add_argument("--duration", type=float, default=0.35, help="Capture window in seconds (default: 0.35)")
    parser.add_argument("--field", choices=["space", "mark"], default="space", help="Timing parameter determining bits (default: space)")
    parser.add_argument("--output", default="ir_buttons.json", help="JSON file to save captured results (default: ir_buttons.json)")
    parser.add_argument("--power", action="store_true", help="Enable the 3.3V V+ user power supply to power the IR receiver")
    args = parser.parse_args()

    dwf = load_dwf_library()
    hdwf, actual_sample_rate, num_samples = setup_device(dwf, args.pin, args.rate, args.duration, args.power)

    buttons_data = {}
    console.print("\n" + "="*60, style="bold green")
    console.print("  Digilent WaveForms IR Decoder Initialized (uv-compatible)", style="bold green")
    console.print("  Press Ctrl+C to stop acquisition loops and write reports.", style="bold green")
    console.print("="*60 + "\n", style="bold green")

    try:
        while True:
            try:
                button_name = input("\nEnter button name (or press Enter to finish and save): ").strip()
            except EOFError:
                break
            if not button_name:
                break
                
            while True:
                samples = acquire_data(dwf, hdwf, num_samples, button_name)
                if samples == "CANCEL":
                    choice = input("Option: (r)etry this button, (s)kip it, or (e)xit? [r/s/e]: ").strip().lower()
                    if choice == 'e':
                        raise KeyboardInterrupt
                    elif choice == 's':
                        break
                    else:
                        continue # retry capture
                        
                result = process_samples(samples, args.pin, actual_sample_rate, args.field)
                if not result or not result["packets"]:
                    console.print("[bold red]Could not process signal. Please try again.[/bold red]")
                    choice = input("Option: (r)etry this button, (s)kip it? [r/s]: ").strip().lower()
                    if choice == 's':
                        break
                    else:
                        continue
                        
                # Print parsed info
                console.print(f"\n[bold cyan]--- Decoded: {button_name} ---[/bold cyan]")
                for p in result["packets"]:
                    idx = p["packet_index"]
                    bit_str = p["bits"]
                    console.print(f"  [bold]Packet {idx + 1}:[/bold]")
                    if p["leader_mark_us"] and p["leader_space_us"]:
                        console.print(f"    Leader: Mark={p['leader_mark_us']:.1f}us, Space={p['leader_space_us']:.1f}us")
                    console.print(f"    Bits ({len(bit_str)}): [green]{bit_str}[/green]")
                    if bit_str:
                        console.print(f"    Hex (LSB-first): [yellow]{to_hex_lsb(bit_str)}[/yellow]")
                        console.print(f"    Hex (MSB-first): [yellow]{to_hex_msb(bit_str)}[/yellow]")
                    console.print(f"    ESPHome Raw timings count: {len(p['raw_timings'])}")
                
                # Store
                buttons_data[button_name] = {
                    "threshold_us": result["threshold_us"],
                    "packets": [
                        {
                            "bits": p["bits"],
                            "hex_lsb": to_hex_lsb(p["bits"]) if p["bits"] else "",
                            "hex_msb": to_hex_msb(p["bits"]) if p["bits"] else "",
                            "raw_timings": p["raw_timings"]
                        }
                        for p in result["packets"]
                    ]
                }
                break # break retry loop to go to next button
                
    except KeyboardInterrupt:
        console.print("\n[bold yellow]Exiting interactive loop.[/bold yellow]")

    finally:
        # Turn off power supply if enabled
        if 'args' in locals() and args.power:
            console.print("[yellow]Disabling user power supplies...[/yellow]")
            dwf.FDwfAnalogIOEnableSet(hdwf, ctypes.c_int(0))
        # Close device
        console.print("\nClosing connection to Digilent device...")
        dwf.FDwfDeviceClose(hdwf)

    if buttons_data:
        # Save JSON
        with open(args.output, "w") as f:
            json.dump(buttons_data, f, indent=4)
        console.print(f"[green]Saved raw button data to {args.output}[/green]")

        # Write Markdown Report
        md_file = "ir_capture_report.md"
        with open(md_file, "w") as f:
            f.write("# IR Remote Capture Report\n\n")
            f.write(f"- **Captured on:** {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"- **Digital Pin:** DIO {args.pin}\n")
            f.write(f"- **Sample Rate:** {actual_sample_rate/1000.0:.2f} kHz\n")
            f.write(f"- **Evaluated Field:** {args.field}\n\n")
            
            f.write("## Captured Buttons\n\n")
            f.write("| Button | Packet | Bits | Hex (LSB-first) | Hex (MSB-first) |\n")
            f.write("|---|---|---|---|---|\n")
            for btn, data in buttons_data.items():
                for p_idx, p in enumerate(data["packets"]):
                    f.write(f"| {btn} | {p_idx+1} | `{p['bits']}` | `{p['hex_lsb']}` | `{p['hex_msb']}` |\n")
            
            f.write("\n## ESPHome Raw Receiver YAML Configuration\n\n")
            f.write("Copy and paste this configuration snippet into your ESPHome configuration:\n\n")
            f.write("```yaml\n")
            f.write("remote_receiver:\n")
            f.write(f"  pin: GPIOX # Update with your ESP GPIO pin\n")
            f.write("  dump: raw\n\n")
            f.write("binary_sensor:\n")
            for btn, data in buttons_data.items():
                for p_idx, p in enumerate(data["packets"]):
                    suffix = f"_pk{p_idx+1}" if len(data["packets"]) > 1 else ""
                    f.write(f"  - platform: remote_receiver\n")
                    f.write(f"    name: \"{btn}{suffix}\"\n")
                    f.write(f"    raw:\n")
                    f.write(f"      code: {p['raw_timings']}\n")
            f.write("```\n")
            
        console.print(f"[green]Generated comprehensive report: {md_file}[/green]")

        # Display Summary Table using Rich
        table = Table(title="Captured IR Buttons Summary", show_header=True, header_style="bold magenta")
        table.add_column("Button", style="cyan")
        table.add_column("Packet", justify="center")
        table.add_column("Bits", style="green")
        table.add_column("Hex (LSB)", style="yellow")
        table.add_column("Hex (MSB)", style="yellow")

        for btn, data in buttons_data.items():
            for p_idx, p in enumerate(data["packets"]):
                table.add_row(
                    btn, 
                    str(p_idx+1), 
                    p["bits"], 
                    p["hex_lsb"], 
                    p["hex_msb"]
                )
        console.print("\n")
        console.print(table)
        console.print("\n")

if __name__ == "__main__":
    main()
