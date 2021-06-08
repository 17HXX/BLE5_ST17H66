import sys
from intelhex import IntelHex
from struct import *
import traceback

def hexf(hexfile):
	try:
		fin = open(hexfile)
	except:
		print('No file opened', hexfile)
		return None
	table =[]
	result = b''
	addr = 0
	addr_flg = 0
	for hexstr in fin.readlines():
		#print(hexstr)
		hexstr = hexstr.strip()
		#print(hexstr)
		size = int(hexstr[1:3],16)
		if int(hexstr[7:9],16) == 4:
			if(len(result)):
				#print(hex(addr))
				#print(addr,result)
				#input()
				table.append([addr,result])
			addr = 	int(hexstr[9:13],16) <<16
			addr_flg = 0
			result = b''
			continue
		if int(hexstr[7:9],16) == 5 or  int(hexstr[7:9],16) == 1:
			#print(hex(addr))
			#print(addr,result)
			table.append([addr,result])
			break
		
		if(addr_flg == 0):
			addr_flg = 1
			addr = addr | (int(hexstr[3:7],16))
		#end if	
		for h in range( 0, size):
			b = int(hexstr[9+h*2:9+h*2+2],16)
			#print(type(b),b,result)
			result += pack('B',b)
		#end if
		#fout.write(result)		
		#result=b''
		#input()
	#end for
	#print(table)
	fin.close()
	return table


def crc16(a,x):
	#print(x)
	if(x is None):
		return a
	#a = 0#xFFFF
	b = 0xa001#0x1021#0xA001
	for byte in x:
		#print(byte, type(a), type(byte)	)
		a ^= byte
		for i in range(8):
			last = a % 2
			a >>= 1
			if last == 1:
				a ^= b
	#s = hex(a).upper()
	#print(s)
	return a
'''	
word      | desc:
0         | flag: "OTAF"
1         | partition number
i*2 + 2   | run address
i*2 + 3   | size
N*2 +2    | data area
'''
# dfl_packer.py flashaddr mode pattern0 pattern1 ...
def printhelp():
	print('Usage: ooo_bin.py fw.hex')
	print('Example: ooo_bin.py mesh_light.hex')

def main(argv):
	if(len(argv) != 2):
		printhelp()
		return
	fname = argv[1]
	
	try:
		ih = IntelHex(fname)
		ih = None
		ih_res = None

	except:
		print('Open hex file failed:', fname)
		printhelp()
		return
		
	ih = hexf(fname)
	ih_res = []
		
	
	fname_bin = fname + '.bin'
	fp = open(fname_bin, 'wb')
	try:
		
		#FW
		lval = []
		for i in range(1024*254):
			lval.append(0xff)
		tag = list('OTAF'.encode())
		for i in range(4):
			lval[i] = 0xff
		partcnt = len(ih) +1
		lval[4] = partcnt & 0xff
		lval[5] = (partcnt>>8) & 0xff
		lval[6] = (partcnt>>16) & 0xff
		lval[7] = (partcnt>>24) & 0xff
		
		
		offset = 0x10
		faddr = 0
		
		#make SelfloadOTAInfoSector:'S','O','I','F'
		soif = []
		for i in range(256):
			soif.append(0xff)
		#flash address
		param = faddr
		print('run is', hex(0xffff00f0), 'faddr is', hex(faddr), 'offset is', offset, 'size is ', len(soif))
		for i in range(len(soif)):
			lval[faddr + 0x1000 + i] = soif[i]

		#flash addr
		lval[offset + 0] = param & 0xff
		lval[offset + 1] = (param>>8) & 0xff
		lval[offset + 2] = (param>>16) & 0xff
		lval[offset + 3] = (param>>24) & 0xff

		#run address
		param = 0xffff00f0
		lval[offset + 4] = param & 0xff
		lval[offset + 5] = (param>>8) & 0xff
		lval[offset + 6] = (param>>16) & 0xff
		lval[offset + 7] = (param>>24) & 0xff

		#partition size
		param = 256
		faddr = faddr + (param+3) &0xfffffffc
		lval[offset + 8] = param & 0xff
		lval[offset + 9] = (param>>8) & 0xff
		lval[offset + 10] = (param>>16) & 0xff
		lval[offset + 11] = (param>>24) & 0xff
		
		#checksum
		param = crc16(0, soif)
		print('crc is ', hex(param))
		lval[offset + 12] = param & 0xff
		lval[offset + 13] = (param>>8) & 0xff
		lval[offset + 14] = (param>>16) & 0xff
		lval[offset + 15] = (param>>24) & 0xff
		offset += 0x10
		
		
		
		for ihp in ih:
			#flash address
			param = faddr
			print('run is', hex(ihp[0]), 'faddr is', hex(faddr), 'offset is', offset, 'size is ', len(ihp[1]))
			for i in range(len(ihp[1])):
				lval[faddr + 0x1000 + i] = ihp[1][i]

			#flash addr
			lval[offset + 0] = param & 0xff
			lval[offset + 1] = (param>>8) & 0xff
			lval[offset + 2] = (param>>16) & 0xff
			lval[offset + 3] = (param>>24) & 0xff

			#run address
			param = ihp[0]
			lval[offset + 4] = param & 0xff
			lval[offset + 5] = (param>>8) & 0xff
			lval[offset + 6] = (param>>16) & 0xff
			lval[offset + 7] = (param>>24) & 0xff

			#partition size
			param = len(ihp[1])
			faddr = faddr + (param+3) &0xfffffffc
			lval[offset + 8] = param & 0xff
			lval[offset + 9] = (param>>8) & 0xff
			lval[offset + 10] = (param>>16) & 0xff
			lval[offset + 11] = (param>>24) & 0xff
			
			#checksum
			param = crc16(0, ihp[1])
			print('crc is ', hex(param))
			lval[offset + 12] = param & 0xff
			lval[offset + 13] = (param>>8) & 0xff
			lval[offset + 14] = (param>>16) & 0xff
			lval[offset + 15] = (param>>24) & 0xff
			offset += 0x10
			
			
		lval = lval[:(faddr + 0x1000)]
		fp.write(bytes(lval))
		fp.close()
	except:
		traceback.print_exc()
		print('Open hex file failed:', fname)
		printhelp()
	
	
	
if __name__ == '__main__':
	main(sys.argv)
