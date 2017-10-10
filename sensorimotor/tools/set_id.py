#!/usr/bin/python

import serial
import argparse
import sys


default_port = '/dev/ttyUSB1'
baudrate = 1000000


def check_board_id(bid):
	if int(bid) > 127 or int(bid) < 0:
		print("Board id '{0}' is out of range 0..127".format(bid))
		return False
	return True


def eat(ser):
	b = ser.read()
	while b:
#		print("dump: {0}".format(b)),
		b = ser.read()


def ping(ser, board_id):
	eat(ser)
	ser.write(chr(255))      # sync 1
	ser.write(chr(255))      # sync 2
	ser.write(chr(224))      # send ping 0xE0
	ser.write(chr(board_id)) # send id

	# check for response
	resp = ser.read() # read response byte
	if resp and ord(resp) == 225: #0xE0
		bid = ser.read() # read id
		if bid and ord(bid) == board_id:
			return True
	return False


def set_id(ser, board_id, new_id):
	eat(ser)
	ser.write(chr(255))      # sync 1
	ser.write(chr(255))      # sync 2
	ser.write(chr(112))      # send set_id command 0x70
	ser.write(chr(board_id)) # send old id
	ser.write(chr(new_id))   # send new id
	# TODO this command should get a checksum

	# check for response
	resp = ser.read() # read response byte
	if resp and ord(resp) == 113: #0x71
		bid = ser.read() # read id
		if bid and ord(bid) == new_id:
			return True
	return False


def main():
	parser = argparse.ArgumentParser()
	parser.add_argument('-b', '--board' , default='')
	parser.add_argument('-p', '--port'  , default=default_port)
	parser.add_argument('-n', '--newid' )
	args = parser.parse_args()

	# open serial communication
	with serial.Serial(args.port, baudrate, timeout=0.1) as ser:
		print("Connected to port {0}\nwith baudrate {1}.\n".format(ser.port, ser.baudrate))

		if args.board and check_board_id(args.board):
			bid = int(args.board)
			print("Sending ping to board id {0}.".format(bid))
			if ping(ser, bid):
				print("Board {0} responded.".format(bid))
				if args.newid and check_board_id(args.newid):
					newid = int(args.newid)
					if ping(ser, newid):
						print("Setting board id failed. ID already in use.")
						return
					if set_id(ser, bid, newid):
						print("Succesfully set new board id from '{0}' to '{1}'".format(bid,newid))
					else:
						print("Setting board id failed.")

			else:
				print("No response.")
		else:
			try:
				print("searching for connected boards...")
				num_boards = 0
				for b in range(128):
					sys.stdout.write("\r{0}".format(b))
					sys.stdout.flush()
					if ping(ser, b):
						num_boards += 1
						print("\rboard {0} responded.".format(b))
			except KeyboardInterrupt:
				print("Aborted.")

			finally:
				if num_boards > 0:
					print("\r{} boards detected.\n".format(num_boards))


	print("\n____\nDONE.")


if __name__ == "__main__": main()

