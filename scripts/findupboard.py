#!/usr/bin/python
from BaseHTTPServer import BaseHTTPRequestHandler,HTTPServer
import nmap
import netifaces as ni

PORT_NUMBER = 8080

#This class will handles any incoming request from
#the browser

nm = nmap.PortScanner()
ni.ifaddresses('eth1')
ip = ni.ifaddresses('eth1')[2][0]['addr']

class myHandler(BaseHTTPRequestHandler):

        def get_hosts(self):
            nm.scan(hosts='192.168.10.*', arguments='-p 22 --open')
            output = []
            for host in nm.all_hosts():
                if host == ip:
                    continue
                output.append(host)

            return output

	#Handler for the GET requests
	def do_GET(self):
		self.send_response(200)
		self.send_header('Content-type','text/html')
		self.end_headers()
		# Send the html message
                self.wfile.write("Please wait while scanning... <br /><br />")

                hosts = self.get_hosts()
                self.wfile.write("Possible upboard ips:<br />")
                for h in hosts:
                    self.wfile.write("{}<br />".format(h))
		return

try:
	#Create a web server and define the handler to manage the
	#incoming request
	server = HTTPServer(('', PORT_NUMBER), myHandler)
	print 'Started httpserver on port ' , PORT_NUMBER

	#Wait forever for incoming htto requests
	server.serve_forever()

except KeyboardInterrupt:
	print '^C received, shutting down the web server'
	server.socket.close()

