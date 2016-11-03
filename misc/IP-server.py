#!/usr/bin/env python

"""
This code is derived from https://gist.github.com/bradmontgomery/2219997
for the HTTP server components
and http://stackoverflow.com/a/33245570 for the IP address detection.
"""


from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
import os

class S(BaseHTTPRequestHandler):
    interface = 'wlp2s0'

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
        ip = f.read() or "Detection failed"

        return ip


        
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