#!/bin/bash

cd $(rospack find oars_pkg)/gui
python -m SimpleHTTPServer
