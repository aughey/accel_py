import smbus
import time
import math
import struct
import datetime
import sys

bus = smbus.SMBus(0)

address_xm = 0x1d
address_g = 0x6b

def write_accel(reg,value):
  bus.write_byte_data(address_xm,reg,value)

def write_check_accel(reg,value):
  write_accel(reg,value)
  res = bus.read_byte_data(address_xm,reg)
  if res != value:
    print "Accel Register " + format(reg,'02x') + " should be " + format(value,'02x') + " but it is " + format(res,'02x')
    abort()


def read16(address,reg):
  low = bus.read_byte_data(address,reg)
  high = bus.read_byte_data(address,reg+1)
  high <<= 8
  high |= low
  if high > 32867:
    high = -(65536 - high)
  return high
  

ACCEL_MG_LSB_2G = 0.061
lsb_g = ACCEL_MG_LSB_2G

GYROSCALE_245DPS = 0.00875
gyro_dps_digit = GYROSCALE_245DPS

def to_float(values,offset):
  high = values[offset+1]
  high <<= 8
  high |= values[offset]
  if high > 32767:
    high = -(65536 - high)
  return high

def readAccel(data):
  fifo = bus.read_byte_data(address_xm,FIFO_SRC_REG)
  count = fifo & 0x1f
  overrun = False
  if count >= 0x0f:
    overrun = True
  #  print "Warning: accel overrun " + format(fifo,'02x')
  #  print "Count is " + str(count)
  #print format(fifo,'02x')
  #print count

  #for i in range(0,count):
  #  rawdata = bus.read_i2c_block_data(address_xm,0x28,6)
  #  print rawdata
  #  data.append( [
  #    to_float(rawdata,0) * lsb_g / 1000.0,
  #    to_float(rawdata,2) * lsb_g / 1000.0,
  #    to_float(rawdata,4) * lsb_g / 1000.0
  #    ])
  #return

  for i in range(0,count):
    data.append( [
    read16(address_xm,0x28) * lsb_g / 1000.0,
    read16(address_xm,0x2a) * lsb_g / 1000.0,
    read16(address_xm,0x2c) * lsb_g / 1000.0
    ] )
  return overrun  

def readGyro(data):
  fifo = bus.read_byte_data(address_g,FIFO_SRC_REG_G)
  count = fifo & 0x1f
  #if count >= 0x0f:
  #  print "Warning: gyro overrun"

  for i in range(0,count):
    data.append( [
      read16(address_g,0x28) * gyro_dps_digit,
      read16(address_g,0x2a) * gyro_dps_digit,
      read16(address_g,0x2c) * gyro_dps_digit
    ] )
  
  return data

CTRL_REG0_XM=0x1f
CTRL_REG1_XM=0x20
CTRL_REG2_XM=0x21
CTRL_REG5_XM=0x24
CTRL_REG6_XM=0x25
CTRL_REG7_XM=0x26

CTRL_REG1_G=0x20
CTRL_REG4_G=0x23
CTRL_REG5_G=0x24

FIFO_CTRL_REG_G=0x2e
FIFO_CTRL_REG=0x2e

FIFO_SRC_REG=0x2f
FIFO_SRC_REG_G=0x2f

ACCELRANGE_2G=(0x00 << 3)
GYROSCALE_245DPS=0x0

# Reboot
bus.write_byte_data(address_g,CTRL_REG5_G,0x80)

# Reboot
write_accel(CTRL_REG0_XM,0x80)

time.sleep(1)

#write_check_accel(CTRL_REG1_XM, 0x1f) # 3hz
#write_check_accel(CTRL_REG1_XM, 0x4f) # 25hz
write_check_accel(CTRL_REG1_XM, 0x67) # 100hz
#write_check_accel(CTRL_REG1_XM, 0x77) # 200hz
#write_check_accel(CTRL_REG1_XM, 0x87) # 400hz

bus.write_byte_data(address_xm,CTRL_REG7_XM,0x00) # No filtering
bus.write_byte_data(address_xm,CTRL_REG2_XM,ACCELRANGE_2G) # 2G accel rate
bus.write_byte_data(address_g,CTRL_REG1_G,0x0f)  # Gyro 95Hz on XYZ
bus.write_byte_data(address_g,CTRL_REG4_G,GYROSCALE_245DPS)  # rate

bus.write_byte_data(address_g,CTRL_REG5_G,0x40)  # Enable fifo for G
bus.write_byte_data(address_g,FIFO_CTRL_REG_G,0x7f)  # stream to fifo

bus.write_byte_data(address_xm,CTRL_REG0_XM,0x40)  # enable fifo for xm
bus.write_byte_data(address_xm,FIFO_CTRL_REG,0x7f)  # stream to fifo

def format3(a):
  return '{0:+1.2f} {1:+1.2f} {2:+1.2f}'.format(a[0],a[1],a[2])

def json3(a):
  return '[ {0},{1},{2} ]'.format(a[0],a[1],a[2])

start = datetime.datetime.now()
accel_count=0
gyro_count=0
while True:
  sys.stdout.flush()
  now = datetime.datetime.now()
  diff = (now - start).total_seconds()
  if diff > 2.0:
    #print "Accel " + str(accel_count / diff) + " Hz"
    #print "Gyro " + str(gyro_count / diff) + " Hz"
    start = now
    accel_count = 0
    gyro_count = 0

  accel_data = []
  accel_overrun = readAccel(accel_data)
  accel_count += len(accel_data)
  #print len(accel_data)
  for a in accel_data:
    g = math.sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2])
    print '{{ "accel": {{ "overrun": {0}, "data": {1} }} }}'.format('true' if accel_overrun else 'false',json3(a))
    #print format3(a)," ",g
  continue

  gyro_data = []
  readGyro(gyro_data)
  gyro_count += len(gyro_data)
  for gyro in gyro_data:
    print json3(gyro)
    #print format3(gyro)
  
