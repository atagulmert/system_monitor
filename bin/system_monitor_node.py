#! /usr/bin/env python2

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from system_monitor.msg import *
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

class Monitor():

	def __init__(self):
		self._pub = rospy.Publisher('~diagnostics', Diagnostic, queue_size=1)
		self._diag_net = DiagnosticNET()
		self._diag_mem = DiagnosticMEM()
		self._diag_cpu_temp = DiagnosticCPUTemperature()
		self._diag_cpu_usa = DiagnosticCPUUsage()
		self._diag_hdd = DiagnosticHDD()
		r = rospy.get_param("rate_param", 0.5)
		self._rate = rospy.Rate(r)

	#Update network values
	def update_net_values(self, status):
		self._diag_net.name = status.name
		self._diag_net.message = status.message
		self._diag_net.hardware_id = status.hardware_id
		net_status = NetStatus()
		net_status.status = status.values[0].value
		net_status.time = float(status.values[1].value)
		ifaces = (len(status.values) - 2) / 10
		for i in xrange(0, ifaces):
			inter = Interface()
			inter.name = status.values[2+10*i].value
			inter.state = status.values[3+10*i].value
			inter.input = float(status.values[4+10*i].value[:-6])
			inter.output = float(status.values[5+10*i].value[:-6])
			inter.mtu = int(status.values[6+10*i].value)
			inter.received = float(status.values[7+10*i].value)
			inter.transmitted = float(status.values[8+10*i].value)
			inter.collisions = int(status.values[9+10*i].value)
			inter.rxError = int(status.values[10+10*i].value)
			inter.txError = int(status.values[11+10*i].value)
			net_status.interfaces.append(inter)
		self._diag_net.status = net_status
		self.publish_info()

	#Update memory values
	def update_mem_values(self, status):
		self._diag_mem.name = status.name
		self._diag_mem.message = status.message
		self._diag_mem.hardware_id = status.hardware_id
		mem_status = MEMStatus()
		mem_status.time = float(status.values[1].value)
		mem_status.totalM = int(status.values[-3].value[:-1])
		mem_status.usedM = int(status.values[-2].value[:-1])
		mem_status.freeM = int(status.values[-1].value[:-1])
		names = ['Physical','Swap']
		for i in xrange(0, 2):
			mem = Memory()
			mem.name = names[i]
			mem.total = int(status.values[3+5*i].value[:-1])
			mem.used = int(status.values[4+5*i].value[:-1])
			mem.free = int(status.values[5+5*i].value[:-1])
			mem_status.memories.append(mem)
		mem = Memory()
		mem.name = "Physical w/o buffers"
		mem.used = int(status.values[6].value[:-1])
		mem.free = int(status.values[7].value[:-1])
		mem_status.memories.append(mem)
		self._diag_mem.status = mem_status
		self.publish_info()

	#Update cpu_temp values
	def update_cpu_temp_values(self, status):
		self._diag_cpu_temp.name = status.name
		self._diag_cpu_temp.message = status.message
		self._diag_cpu_temp.hardware_id = status.hardware_id
		aux_temp = CPUTemperatureStatus()
		aux_temp.status = status.values[0].value
		aux_temp.time = float(status.values[1].value)
		for i in range(2, len(status.values)):
			core = CoreTemp()
			core.id = i - 2
			core.temp = float(status.values[i].value[:-4])
			aux_temp.cores.append(core)
		self._diag_cpu_temp.status = aux_temp
		self.publish_info()

	#Update cpu_usage values
	def update_cpu_usa_values(self, status):
		self._diag_cpu_usa.name = status.name
		self._diag_cpu_usa.message = status.message
		self._diag_cpu_usa.hardware_id = status.hardware_id
		aux_usa = CPUUsageStatus()
		len_values = len(status.values)
		num_cores = (len_values - 6)/6
		aux_usa.status = status.values[0].value
		aux_usa.time = float(status.values[1].value)
		aux_usa.load_status = status.values[len_values - 4].value
		aux_usa.load_avg1 = float(status.values[len_values - 3].value[:-1])
		aux_usa.load_avg5 = float(status.values[len_values - 2].value[:-1])
		aux_usa.load_avg15 = float(status.values[len_values - 1].value[:-1])
		for i in range(0, num_cores):
			core = CoreUsage()
			core.id = i
			core.speed = float(status.values[i +2].value[:-3])
			core.status = status.values[2 + num_cores + 5*i].value
			core.system = float(status.values[3 + num_cores + 5*i].value[:-1])
			core.user = float(status.values[4 + num_cores + 5*i].value[:-1])
			core.nice = float(status.values[5 + num_cores + 5*i].value[:-1])
			core.idle = float(status.values[6 + num_cores + 5*i].value[:-1].replace(",","."))
			aux_usa.cores.append(core)
		self._diag_cpu_usa.status = aux_usa
		self.publish_info()

	#Update hdd values
	def update_hdd_values(self, status):
		self._diag_hdd.name = status.name
		self._diag_hdd.message = status.message
		self._diag_hdd.hardware_id = status.hardware_id
		aux_stat = HDDStatus()
		aux_stat.status = status.values[0].value
		aux_stat.time = float(status.values[1].value)
		aux_stat.space_reading = status.values[2].value
		num_disks = (len(status.values) - 3)/6
		for i in range(0,num_disks):
			disk = Disk()
			disk.id = i + 1
			disk.name = status.values[3 + i * 6].value
			disk.size = float(status.values[4 + i * 6].value[:-1])
			disk.available = float(status.values[5 + i * 6].value[:-1])
			disk.use = float(status.values[6 + i * 6].value[:-1])
			disk.status = status.values[7 + i * 6].value
			disk.mount_point = status.values[8 + i * 6].value
			aux_stat.disks.append(disk)
		self._diag_hdd.status = aux_stat
		self.publish_info()

	#Publish info
	def publish_info(self):
		msg = Diagnostic()
		msg.diagNet = self._diag_net
		msg.diagMem = self._diag_mem
		msg.diagCpuTemp = self._diag_cpu_temp
		msg.diagCpuUsage = self._diag_cpu_usa
		msg.diagHdd = self._diag_hdd
		self._rate.sleep()
		self._pub.publish(msg)


# Print CPU status
def callback(data):

	global memory_it
	global network_it
	global cpu_it
	global memory_usage
	global input_trafic
	global output_trafic
	global average_cpu_temp

	if data.status[0].name.startswith('Memory'):
		memory_usage.append(int(data.status[0].values[4].value[:-1]))
		axs[0].plot(range(0,memory_it),memory_usage)
		plt.draw()
		plt.pause(0.0001)
		memory_it+=1
		#Extract useful data from memory
		mon.update_mem_values(data.status[0])
	elif data.status[0].name.startswith('Network'):
		# This changes with available networks
		#Extract useful data from network
		input_trafic.append(float(data.status[0].values[4].value[:-7]))
		output_trafic.append(float(data.status[0].values[5].value[:-7]))
		axs[1].plot(range(0,network_it),input_trafic)
		axs[2].plot(range(0,network_it),output_trafic)
		plt.draw()
		plt.pause(0.0001)
		network_it+=1
		mon.update_net_values(data.status[0])
	elif data.status[0].name.startswith('CPU Temperature'):
		#Extract useful data from cpu
		cpu_temps=[]
		for i in range(len(data.status[0].values[2:])):
			temp = float(data.status[0].values[2:][i].value.replace('DegC',''))
			if temp < 150 and temp > 0:
				cpu_temps.append(temp)
			else:
				rospy.logwarn('One of the core temperatures is out of range 0DegC - 150DegC. Value is probably not real, removing it.')
		average_cpu_temp.append(np.mean(cpu_temps))
		axs[3].plot(range(0,cpu_it),average_cpu_temp)
		plt.draw()
		plt.pause(0.0001)
		cpu_it+=1

		mon.update_cpu_temp_values(data.status[0])
		mon.update_cpu_usa_values(data.status[1])

	elif data.status[0].name.startswith("HDD Usage"):

		print(data.status[0].values)
		#Extract useful data from disk
		mon.update_hdd_values(data.status[0])


if __name__ == '__main__':
	rospy.init_node('system_monitor_node')
	mon = Monitor()

	plt.ion()
	fig, axs = plt.subplots(4)

	memory_it = 1
	network_it = 1
	cpu_it = 1
	memory_usage = []
	input_trafic = []
	output_trafic = []
	average_cpu_temp = []
	axs[0].set_title('Memory Usage (MB)')
	axs[1].set_title('Input Trafic (MBps)')
	axs[2].set_title('Output Trafic (MBps)')
	axs[3].set_title('CPU Tempature (DegC)')

	for i in range(4):
		axs[i].tick_params(
		axis='x',          
		which='both',      
		bottom=False,      
		top=False,         
		labelbottom=False)
		axs[i].set_xlabel('Time')
	plt.tight_layout()
	rospy.Subscriber('/diagnostics', DiagnosticArray, callback)
	#rospy.spin()
	plt.show(block=True)
