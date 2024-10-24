import subprocess

with open("index.html", "r") as html_file:
    with open("WebPage.h", "w") as h_file:
        h_file.write('const char PAGE_MAIN[] PROGMEM = R"=====(')
        h_file.write(html_file.read())
        h_file.write(')=====";')

compiled = subprocess.run(["arduino-cli", "compile", "--fqbn",
               "esp32:esp32:uPesy_wroom", "BoatCode.ino"])
if (compiled.returncode):
    print("Compilation failed!")
    exit(1)

board_list = subprocess.run(["arduino-cli", "board", "list"], capture_output=True)
board_list = board_list.stdout.decode("utf-8")
board_list = board_list.split("\n")
board_list = [line for line in board_list if "serial" in line]
board_list = [line.split(" ")[0] for line in board_list]
ESP32_board = board_list[0]
print("Uploading to " + ESP32_board)
subprocess.run(["arduino-cli", "upload", "-p", ESP32_board, "--fqbn",
                "esp32:esp32:uPesy_wroom", "BoatCode.ino"])
