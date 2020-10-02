import keyboard
import time

def handle_press(e):
  print(e.name, e.scan_code, e.time)


def main():
  while True:
    cmd = input()
    pan, tilt = split(cmd, ',')

if __name__ == '__main__':
  main()