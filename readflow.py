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

class PolyScienceCA10:
	"""
	Uses python3-serial to communicate with a PolyScience CA10 egirgerated recirculating chiller.

	with PolyScienceCA10('/dev/tty...') as s: s.GetSetPoint()

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
			s = PolyScienceCA10(port)
			s._open()
			yield s
		finally:
			if s:
				s._close()

	def write(self, msg):
		"""Encode into ASCII and write over serial"""
		self._serial.write(msg.encode('ascii'))

	def readline(self):
		"""Read a line and decode from ASCII, check the LRC, and return the message"""
		# Reading is painfully slow (seconds for a response), not sure why
		# It is CR based not LF or CRLF, so it's not a timeout problem
		r = self._serial.read_until('\r')
		z = r.decode('ascii').strip('\r')

		return z

	# --------------------------------------------------------
	# Functions

	def PowerOn(self):
		self.write('SO1\r')
		ret = self.readline()
		if ret == '!':
			return
		else:
			raise ValueError("Failed to power on")

	def PowerOff(self):
		self.write('SO0\r')
		ret = self.readline()
		if ret == '!':
			return
		else:
			raise ValueError("Failed to power off")

	def PowerStatus(self):
		self.write('RW\r')
		ret = self.readline()
		return ret[0] == '1'

	def GetSetPoint(self):
		self.write('RS\r')
		ret = self.readline()
		return float(ret)

	def SetSetPoint(self, t):
		self.write('SS%.1f\r' % t)
		ret = self.readline()
		if ret == '!':
			return
		else:
			raise ValueError("Failed to set the set point to %.1f" % t)

	def GetTemperature(self):
		"""Read temperature in whatever units is set"""
		self.write('RT\r')
		ret = self.readline()
		return float(ret)

	def GetTemperatureUnits(self):
		"""Read temperature units: 'C' == celcius, 'F' == Fahrenheit"""
		self.write('RU\r')
		ret = self.readline()
		return ret[0]

	def GetPressurePSI(self):
		"""Read in pounds per square inch (PSI)"""
		self.write('RP\r')
		ret = self.readline()
		return float(ret)

	def GetPressureKPA(self):
		"""Read in kilopascals (kPa)"""
		self.write('RK\r')
		ret = self.readline()
		return float(ret)

	def GetFlowGPM(self):
		"""Gallons per minute"""
		self.write('RG\r')
		ret = self.readline()
		return float(ret)

	def GetFlowLPM(self):
		"""Liters per minute"""
		self.write('RL\r')
		ret = self.readline()
		return float(ret)

	def GetTotalPower(self):
		v = self.GetPowerVoltage()
		a1 = self.GetCompressorAmps()
		a2 = self.GetPumpAmps()

		return v*(a1+a2)

	def GetPowerVoltage(self):
		self.write('RV\r')
		ret = self.readline()
		return float(ret)

	def GetCompressorAmps(self):
		self.write('RCA\r')
		ret = self.readline()
		return float(ret)

	def GetPumpAmps(self):
		self.write('RPA\r')
		ret = self.readline()
		return float(ret)

	def GetFluidLevelStatus(self):
		"""Get fluid leve status: True == good, Fale == low"""
		self.write('RX\r')
		ret = self.readline()
		# Good is 0, low is 1
		return ret[0] == '0'

	def GetFluidLevel(self):
		self.write('RFL\r')
		ret = self.readline()
		return float(ret)

	def GetAmbientTemperature(self):
		"""Get ambient tempreature in whatever units is set (see GetTemperatureUnits())"""
		self.write('RA\r')
		ret = self.readline()
		return float(ret)

	def GetAmbientHumidity(self):
		"""Get ambient relative humidity in percent"""
		self.write('RRH\r')
		ret = self.readline()
		return float(ret)

	def GetAmbientPressure(self):
		"""Get ambient barometric pressure, I think this is in hectopascals (hPa)"""
		self.write('RBP\r')
		ret = self.readline()
		return float(ret)

	def GetOperatingHours(self):
		"""Get operating hours"""
		self.write('ROC\r')
		ret = self.readline()
		return float(ret)

	def GetOperatingCycles(self):
		"""Get on/off cycles"""
		self.write('RCC\r')
		ret = self.readline()
		return int(ret)

	def GetSystemFault(self):
		"""Get system fault (0 == system ok)"""
		self.write('RF\r')
		ret = self.readline()
		return int(ret)

class PowerCycler1:
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
			s = PowerCycler1(port)
			s._open()
			yield s
		finally:
			if s:
				s._close()

	def write(self, msg):
		"""Encode into ASCII and write over serial"""
		if msg.endswith('\n'):
			m = msg.encode('ascii')
		else:
			m = (msg + '\n').encode('ascii')

		self._serial.write(m)

	def readline(self):
		"""Read a line and decode from ASCII, and return the message"""
		r = self._serial.readline()
		z = r.decode('ascii').strip('\n')

		return z

	# --------------------------------------------------------
	# Functions

	def Cycle(self, N):
		"""
		Cycle the power for relay N (0, 1, 2, or 3)
		"""

		if N < 0 or N > 3:
			raise ValueError("Can only cycle power on relays 0, 1, 2, or 3; got %s" % N)

		return self.write("cycle[%d]" % N)

def TestPolyScienceCA10():
	with PolyScienceCA10.open('/dev/ttyUSB0') as s:
		ret = s.PowerStatus()
		if ret:
			print('Power is On')
		else:
			print('Power is Off')

		f = s.GetSystemFault()
		print('System Fault %d' % f)

		f = s.GetPowerVoltage()
		print('Power voltage is %.1f volts' % f)

		f = s.GetCompressorAmps()
		print('Compressor Amps %.1f Amps' % f)

		f = s.GetPumpAmps()
		print('Pump Amps %.1f Amps' % f)

		f = s.GetTotalPower()
		print('Power Consumption %.1f Watts' % f)


		f = s.GetOperatingHours()
		print('Operating Hours %.1f hours' % f)

		f = s.GetOperatingCycles()
		print('Operating Hours %d cycles' % f)

		ret = s.GetFluidLevelStatus()
		if ret:
			print('Fluid Level Good')
		else:
			print('Fluid Level Low')

		f = s.GetFluidLevel()
		print('Fluid Level %.1f' % f)


		f = s.GetAmbientTemperature()
		print('Ambient Temperatures %.1f' % f)

		f = s.GetAmbientHumidity()
		print('Ambient Humidity %.1f %%' % f)

		f = s.GetAmbientPressure()
		print('Ambient Pressure %.1f hPa' % f)


		f = s.GetSetPoint()
		print('Set point %.2f' % f)

		f = s.GetTemperature()
		print('Temp %.2f' % f)

		f = s.GetPressurePSI()
		print('Pressure %.2f PSI' % f)

		f = s.GetFlowLPM()
		print('Flow %.2f LPM' % f)

def TestSmartTrak50():
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

if __name__ == '__main__':
	# Test use by querying for all the things

	#TestPolyScienceCA10()
	TestSmartTrak50()

