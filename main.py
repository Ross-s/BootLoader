import os

if __name__ == "__main__":
    # Check if boot.py exists on SD card and execute it
    boot_file_path = "/sd/main.py"
    try:
        if "boot.py" in os.listdir("/sd"):
            print("\nExecuting boot.py from SD card...")
            with open(boot_file_path, "r") as f:
                exec(f.read())
        else:
            print("\nNo boot.py found on SD card")
    except Exception as e:
        print("Error executing SD card boot.py:", e)
