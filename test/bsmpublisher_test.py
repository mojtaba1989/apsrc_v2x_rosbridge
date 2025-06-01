import struct
import socket
import time
import numpy as np
import sys

dataBSM = b'\x03\x81\x00@\x03\x80\x81\x85\x00\x14\x80\x81N\r\x04\x05!\x8aZ(\xdd\xfaM\x1b?ZB\x8c\x0b\x98\x98\x80\x00p\x0e\xed\xd1\xfd~\xc7\xcf\x00\x7f\xff\x00\x00\\\x0f\x88\x01h\xc1 \xffE\xbf\xb9\x10>a\x04\x10\x15\xab\xfd#\x05\x93\xa5\xc1\x06\xb4\xbf92\xf7\xff\xfa\x10\x94;\xf3\x079_\xff\xa1\r\xec?~R\xd5\xff\xfa\x10q\\\x00\xed&?\xff\xa1\x03\xad?k\x11Q\xff\xfa\x10\x82\xcb\xfb5\x14?\xff\xa1\x05)\xbf)\xf1\xaf\xff\xfa\x10w;\xf8!\x11\xbf\xff\xaf\xff\xec\x80@\x01 \x00\x02.Hz+I\xe5\x80\xe1B\x911\x10\x1c\x08\xb5\x80\x82t|\xf7\x164/\xc3\xd9\xfc5]QY\xee\xf4\xe4\xe9;\x95\xbc\x1d(\xf9\x94\x89?\xea\xc1\xc4\xb3\xee\xf41$\x11;\x93\x8c\xdd\x01&\x1b\xb0W\xb6\xdb\x16\x16j\xb6_7\xb2\xed\xe3X\xc1\x9e\xd1\xea\xa3\xc4\xa0\xa1'
dataMAP = b'\x03\x81\x00@\x03\x80b\x00\x12_\x08\x00\x00\x05V`\x02\x9a\xe2V\xc3\x83B\x00p\x10 \n\x00\x02 \x00\x00\x00X~\x97\xc6\xd2\xca/\xb5+\x8a@\x88\x00\x05 h\x00\x04\x08\x04\x80\x00\x88\x00\x00\x00\x15\xecy\xf0`\xac`\x8d)"\x90B\x00\x03H\x14\x00\x02\x02\x01\xa0\x00"\x00\x00\x00\x05\x852\x86\x04,\xa4\xdc\x868\xa4\x0e\x00\x01R\x08\x80\x00\xc0P\x01\x82\x00\x02.Hz6\x15\xa4\x1c\x16\xf9h\xcb4\xe8,\x06\xa9\x81\x01\x01\x00\x03\x01\x80;{,(\xe4D\x84\xb50\x80\x80\x00\x08\x11"3DUfw\x88\x99\x11"3D\x99\x88wfUD3"\x11\x12\xab4\x00\x00$\x95\xa0\x00\x84\x000\x00\x01\x05\x80\x01\x82\x80\x00\x80\x01\x83\x80\x00\x80\x02@`\x80\x00\x80\x02@p\x80\x00\x80\x03r\x93\xc1\x80\x00\x81\x83\x0bZS\xde\xdaJi@\xf4A\x9a\x81"wP\x8c\xde\xaf\xb9\xf1\xd66\xady\x03\xf6k\x1f\xa8\xa2k\xcf\x80\x83\xc0\x9b#fB\x8e\xd0\xe58\x9b\xa4\xb5{\xc7&p\xbc\x00\x9d\x19\xefP\xf4j\xfd\x0b:+\xfe\x8c\xaa\xd6\x89\xe5\xff\xc7\x7f\x8c\xa4\xa2\xfaD\x1a\x00F\x1d!\xc2\x9a\x91!\xe9)\x07\xcf\t\xdbT\x07,\xb6\x01\x01\xf9'
dataSPaT = b'\x03\x81\x00@\x03\x80x\x00\x13u\x00\x00*\xb3\x01\x00\x00\x05\x00\x10c\x03e\xe1(\x80\x00\x0f(\x87\x00\x19(\x8d\xe02\x00\x101\x81\xb2\xf0\x94@\x00\x05\x14E0\n\x14H\xb02\x00\x0c\x19\x80\xd9<J \x00\x01J!\x10\x07\x8a!0\x1a@\x08\x0c\xc0l\x9e%\x10\x00\x00\xf5\x10(\x01\x95\x12d\x05\xf0\x05\x06`6O\x12\x88\x00\x00\xe2\x888\x01\xba\x88\x8a\x04\xb0\x03\x03\x18\x1bO\x89D\x00\x00mD\xa7\x00\xc5D\xde\x02\xd0P\x01\x82\x00\x02.Hz+L\xb4\x1c\x16\xf9h\xcb4\xe8,\x06\xa9\x81\x01\x01\x00\x03\x01\x80;{,(\xe4D\x84\xb50\x80\x80\x00\x08\x11"3DUfw\x88\x99\x11"3D\x99\x88wfUD3"\x11\x12\xab4\x00\x00$\x95\xa0\x00\x84\x000\x00\x01\x05\x80\x01\x82\x80\x00\x80\x01\x83\x80\x00\x80\x02@`\x80\x00\x80\x02@p\x80\x00\x80\x03r\x93\xc1\x80\x00\x81\x83\x0bZS\xde\xdaJi@\xf4A\x9a\x81"wP\x8c\xde\xaf\xb9\xf1\xd66\xady\x03\xf6k\x1f\xa8\xa2k\xcf\x80\x83\x07\x7fT\xb0(\x13A\xcey\xc0\xe7Uom\x81\x92\xf1\x19\xd3J\x00\x02\xacBDG\x11\xce\t9su}\x8ds\xc6\x0c}\x18\x83\xce\xad!\xe8\xb2[\xbc)D\x03\xc2\x8ao\x19\\\x86\x83\x8e\xad\xc8\x86\xc7\x19\r'
# dataSPaT = "03801d00131a439eff00800009d000096070100104340e460e78001022807110"
# dataSPaT = bytearray.fromhex(dataSPaT)

# dataMAP = "03808201f3001281ef38003000200002004ed28932397d396902dc2058022800000500040dc464051c061312381499a46fc765f8dd8ec451b9519ee36c334606d546d40d8fce181b301e18280e024050116010a0000004000637e78fe86fd76178df7ec231bfc190e37e7b24c1014045806480000040000961d5a1024b5ac702405d2d481f4040880960212000001000025875f858e2d6a9c07f974b920828102002580a480000040000961d3a1b98b5a9702405d2f581ff040780960312000000400005872d88602cd2ac0530203804b01cd000000800022bca73da81581f1fe18ad0af01a655ca882c704060096041a000001000045794e79d62b03a3fc895a13e034cab96105a4080b012c093400000200008af29cf0e65607c7f912b41b405e1572c20ba01014022014840000001161c11e250b50bd022a5a38180f42da9740ac0805a100000004586f47a192d42f407a168e2a03fcb6a7d029a2018840000001161b8deefcb50dd01905a38180e92daa3c0b18806a1000000085aaaf7df62c2514005966bc603a4b03ed010a581b481582c041c0d30807290000000257d9f8a492c008443595fe164788403d8800000022bc6dc3731582b9fb50ad0c0fef655ca07f2d10106200000008af1d70a80560a67f172b42bbfb8157281fd1040458800000022bc7a41f4158275fc30ad0aefeca55ca87f44"
# dataMAP = bytes.fromhex(dataMAP)
dataRSA = "0b030f01ac10010c04019800e052534126001b233f51c0dae0000000000000000000000000000000001fffea0036845bed9d7d0f306a60"
dataRSA = bytes.fromhex(dataRSA)


datalist = []
class dataset:
    def __init__(self, name, data):
        self.name = name
        self.data = data
        datalist.append(self)





message_set = []
max_iter = 1
iter_flag = False
freq = 1

commands = [arg for arg in sys.argv[1:] if "-" in arg]
for arg in commands:
    if not arg in ['--bsm', '-b','--map',
                    '-m','--spat', '-s',
                    '--iter', '-i',
                    '--freq', '-f',
                    '--rsa', '-r'] :
        print("bad argument")
        exit(1)


for idx, arg in enumerate(sys.argv):
    if arg in ['--bsm', '-b']:
        dataset('BSM', dataBSM)
        del sys.argv[idx]

for idx, arg in enumerate(sys.argv):
    if arg in ['--map', '-m']:
        dataset('MAP', dataMAP)
        del sys.argv[idx]

for idx, arg in enumerate(sys.argv):
    if arg in ['--spat', '-s']:
        dataset('SPat', dataSPaT)
        del sys.argv[idx]

for idx, arg in enumerate(sys.argv):
    if arg in ['--rsa', '-r']:
        dataset('RSA', dataRSA)
        del sys.argv[idx]

for idx, arg in enumerate(sys.argv):
    if arg in ['--iter', '-i']:
        max_iter = max([int(sys.argv[idx+1]), 1])
        del sys.argv[idx]
        del sys.argv[idx]
        iter_flag = True

for idx, arg in enumerate(sys.argv):
    if arg in ['--freq', '-f']:
        freq = max([int(sys.argv[idx+1]), 1])
        del sys.argv[idx]
        del sys.argv[idx]




UDP_IP = "127.0.0.1"
UDP_PORT = 1551


if (not datalist):
    dataset('MAP', dataMAP)
    dataset('SPat', dataSPaT)
    dataset('BSM', dataBSM)

sock = socket.socket(socket.AF_INET, # Internet
            socket.SOCK_DGRAM) # UDP
current_iter = 0
while current_iter < max_iter:
    try:
        i = np.random.randint(0, len(datalist))
        print(datalist[i].name)
        print(bytes.hex(datalist[i].data))
        sock.sendto(datalist[i].data, (UDP_IP, UDP_PORT))
        time.sleep(1/freq)
        if (iter_flag):
            current_iter += 1
    except KeyboardInterrupt:
        sock.close()
        break

