#!/usr/bin/env python

"""
This program responds to requests to port 80 with the current IP address of the 
computer on the wireless interface defined by S.interface

This is used in concert with dataplicity.com's "wormhole" service to enable
discovery of local IP addresses over the internet in leiu of proper hostname
support on Olin's wifi networks.

This code is derived from https://gist.github.com/bradmontgomery/2219997
for the HTTP server components
and http://stackoverflow.com/a/33245570 for the IP address detection.
"""


from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
import os, platform

class S(BaseHTTPRequestHandler):
    if platform.dist() == ('Ubuntu', '16.04', 'xenial'):
        interface = "wlp2s0"
    else:
        interface = "wlan0"

    def _set_headers(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    def do_GET(self):
        self._set_headers()
        self.wfile.write("""<html><body>
            <h3>Current local wifi IP:</h3>
            {}
            </body></html>""".format(self.get_local_IP()))

    def get_local_IP(self):
        f = os.popen('ifconfig {} | grep "inet\ addr"'.format(self.interface))
        ip = f.read()

        return ip or "Detection failed"


        
def run(server_class=HTTPServer, handler_class=S, port=80):
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    print 'Starting httpd...'
    httpd.serve_forever()

if __name__ == "__main__":
    from sys import argv

    if len(argv) == 2:
        run(port=int(argv[1]))
    else:
        run()