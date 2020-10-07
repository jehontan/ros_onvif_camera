import socket
import struct

def main():
  HOST = '192.168.137.216'
  PORT = 2306

  with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    print('Connected!')
    
    while True:
      txt = input('Pan, Tilt, Zoom >')
      cmds = txt.split(',')
      pan = float(cmds[0])
      tilt = float(cmds[1])
      zoom = float(cmds[2]) if len(cmds)>2 else 0.0

      cmd = struct.pack('fff', pan, tilt, zoom)
      s.send(cmd)

if __name__ == '__main__':
  main()