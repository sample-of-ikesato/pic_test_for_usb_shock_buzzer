#!/usr/bin/env ruby
# -*- coding: utf-8 -*-

$LOAD_PATH.push File.dirname($0)

require "rubygems"
require "serialport"
require 'optparse'

def usage()
  puts "Usage: #{File.basename($0)} [OPTION] USB-DEVICE WAVFILE"
  puts ""
  puts "Options:"
  puts "  -v, --verbose   output verbose format"
  puts "  -d, --debug     debug mode"
  puts "  -h, --help      display this help and exit"
  exit 1
end

debug=false
verbose=false
OptionParser.new {|opt|
  opt.on("--debug") {
    debug=true
  }
  opt.on("--verbose") {
    verbose=true
  }
  opt.on("--help") {
    usage
  }
  opt.parse!(ARGV)
}

usage if ARGV.length < 2
PORT=ARGV.shift
fname = ARGV.shift
fsize = File.size(fname)
sp = SerialPort.new(PORT, 9600, 8, 1, 0) # 8bit, stopbit 1, parity none

count=0
start = last = Time.now
readed = 0
fp = File.open(fname, "rb")
sp.write([1,0].pack("C*"))
dfp = File.open("debug.dat", "wb")
while true
  buf = sp.read(2)
  if buf == 0
    puts "Device detatched!"
    break
  end
  if buf.size < 2
    puts "Error: buffer needs 2 bytes."
    break
  end
  cmd, size = buf.unpack("C*")
  if size > 0
    data = sp.read(size).unpack("C*")
    if data.size < size
      puts "Errror: data needs #{size} bytes"
    end
  end
  if cmd == 2
    count += 1
    rsize = data[0]
    if readed >= fsize
      sp.write([3, 0].pack("C*"))
      #break
      diff = Time.now - start
      printf("\r%02d:%02d.%03d #{readed}/#{fsize} #{rsize} #{count} play back.\n", diff/60, diff%60, (diff * 1000)%1000)
      fp.close

      # loop
      count=0
      start = last = Time.now
      readed = 0
      fp = File.open(fname, "rb")
      sp.write([1,0].pack("C*"))

      # break
      #break
    else
      data = fp.read(rsize)
      readed += data.size
      sp.write([3, data.size].pack("C*") + data)
      if Time.now - last > 0.1
        last = Time.now
        diff = last - start
        printf("\r%02d:%02d.%03d #{readed}/#{fsize} #{rsize}", diff/60, diff%60, (diff * 1000)%1000)
      end
    end
  elsif cmd == 9
    dfp.write(data.pack("C*")) if size > 0
  else
    puts "unrecognize command #{cmd} #{size} #{data}"
  end
end
puts ""
diff = Time.now - start
printf("%02d:%02d.%03d #{readed}/#{fsize} #{rsize} #{count} #{count*32}\n", diff/60, diff%60, (diff * 1000)%1000)
