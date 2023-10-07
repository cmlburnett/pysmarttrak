import contextlib
import serial

class SmartTrak50:
	"""
	Uses python3-serial to communicate with a Sierra Instruments Smart Trak 50 mass flow controler.

	with SmartTrak50('/dev/tty...') as s: s.GetFlow()

	An example usage above.
	"""

	def __init__(self, port):
		self._port = port
		self._serial = None

	@property
	def Port(self): return self._port

	def _open(self):
		if not self._serial:
			self._serial = serial.Serial(self._port, 9600, timeout=5)

	def _close(self):
		self._serial.close()
		self._serial = None

	@contextlib.contextmanager
	def open(port):
		s = None
		try:
			s = SmartTrak50(port)
			s._open()
			yield s
		finally:
			if s:
				s._close()

	@staticmethod
	def CalculateLRC(msg):
		"""Calculate the LRC and append to the message with the CRLF"""
		msg = msg.strip('\r\n')
		m = msg.encode('ascii')

		# 8 bit sum and 2's complement
		s = 256 - (sum(m) % 256)
		h = hex(s).upper()

		return '%s%s\r\n' % (msg, h[2:])

	@staticmethod
	def CheckLRC(msg):
		"""Make sure the LRC provided matches that calculated"""
		lrc = msg[-2:]

		ret = __class__.CalculateLRC(msg[:-2])

		return msg == ret.strip('\r\n')

	def write(self, msg):
		"""Encode into ASCII and write over serial"""
		m = __class__.CalculateLRC(msg).encode('ascii')
		self._serial.write(m)

	def readline(self):
		"""Read a line and decode from ASCII, check the LRC, and return the message"""
		r = self._serial.readline()
		z = r.decode('ascii').strip('\r\n')

		if not self.CheckLRC(z):
			raise ValueError("Got message '%s' and LRC doesn't check out" % r)

		return z[:-2]

	# --------------------------------------------------------
	# Functions

	def GetFlow(self):
		self.write('?Flow')
		ret = self.readline()
		if not ret.startswith('Flow'):
			raise ValueError("Didn't get Flow returned '%s'" % ret)

		flow = ret.split('Flow')[1]
		return float(flow)

	def GetSetpointFlash(self):
		self.write('?Setf')
		ret = self.readline()
		if not ret.startswith('Setf'):
			raise ValueError("Didn't get Setf returned '%s'" % ret)

		setr = ret.split('Setf')[1]
		return float(setr)

	def SetSetpointFlash(self, flow):
		# NOTE: use SetSetpointRAM for routine use, flash burns out so this should be rarely called
		self.write('!Setf%.3f' % flow)
		ret = self.readline()
		if not ret.startswith('Setf'):
			raise ValueError("Didn't get Setf returned '%s'" % ret)

		flow = ret.split('Setf')[1]
		return float(flow)

	def GetSetpointRAM(self):
		self.write('?Setr')
		ret = self.readline()
		if not ret.startswith('Setr'):
			raise ValueError("Didn't get Setr returned '%s'" % ret)

		setr = ret.split('Setr')[1]
		return float(setr)

	def SetSetpointRAM(self, flow):
		self.write('!Setr%.3f' % flow)
		ret = self.readline()
		if not ret.startswith('Setr'):
			raise ValueError("Didn't get Setr returned '%s'" % ret)

		flow = ret.split('Setr')[1]
		return float(flow)

	def GetFullscale(self):
		self.write('?Fscl')
		ret = self.readline()
		if not ret.startswith('Fscl'):
			raise ValueError("Didn't get Fscl returned '%s'" % ret)

		fscl = ret.split('Fscl')[1]
		return float(fscl)

	def GetGasName(self):
		self.write('?Gnam')
		ret = self.readline()
		if not ret.startswith('Gnam'):
			raise ValueError("Didn't get Gnam returned '%s'" % ret)

		gnam = ret.split('Gnam')[1]
		return gnam

	def GetUnits(self):
		self.write('?Unts')
		ret = self.readline()
		if not ret.startswith('Unts'):
			raise ValueError("Didn't get Unts returned '%s'" % ret)

		unts = ret.split('Unts')[1]
		return unts

	def GetVersion(self):
		self.write('?Vern')
		ret = self.readline()
		if not ret.startswith('Vern'):
			raise ValueError("Didn't get Vern returned '%s'" % ret)

		vern = ret.split('Vern')[1]
		return vern

	def GetSerialNumber(self):
		self.write('?Srnm')
		ret = self.readline()
		if not ret.startswith('Srnm'):
			raise ValueError("Didn't get Srnm returned '%s'" % ret)

		srnm = ret.split('Srnm')[1]
		return srnm

	def GetSpan(self):
		self.write('?Span')
		ret = self.readline()
		if not ret.startswith('Gass'):
			raise ValueError("Didn't get Gass returned '%s'" % ret)

		gass = ret.split('Gass')[1]
		return float(gass)

if __name__ == '__main__':
	# Test use by querying for all the things

	with SmartTrak50.open('/dev/ttyUSB0') as s:
		f = s.GetFlow()
		print('Flow %.4f' % f)

		f = s.GetSetpointFlash()
		print('Flash setpoint %.4f' % f)

		f = s.GetSetpointRAM()
		print('RAM setpoint %.4f' % f)

		#f = s.SetSetpointRAM(0.100)
		#print('RAM setpoint %.4f' % f)

		f = s.GetFullscale()
		print('Fullscale %.4f' % f)

		f = s.GetUnits()
		print('Units %s' % f)

		f = s.GetGasName()
		print('Gas Name %s' % f)

		f = s.GetVersion()
		print('Version %s' % f)

		f = s.GetSerialNumber()
		print('SerialNumber %s' % f)

		f = s.GetSpan()
		print('Span %.4f' % f)

