import os, sys
if __name__ == "__main__":
    # Check if main.py exists on SD card and execute it
    main_file_path = "/sd/main.py"
    try:
        sys.path.append("/sd/lib")
        if "main.py" in os.listdir("/sd"):
            print("\nExecuting main.py from SD card...")
            with open(main_file_path, "r") as f:
                exec(f.read())
        else:
            print("\nNo main.py found on SD card")
    except Exception as e:
        print("Error executing SD card main.py:", e)
