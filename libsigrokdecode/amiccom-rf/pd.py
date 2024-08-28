##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2020 Tomas Mudrunka <harvie@github>
##
## Permission is hereby granted, free of charge, to any person obtaining a copy
## of this software and associated documentation files (the "Software"), to deal
## in the Software without restriction, including without limitation the rights
## to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
## copies of the Software, and to permit persons to whom the Software is
## furnished to do so, subject to the following conditions:
##
## The above copyright notice and this permission notice shall be included in all
## copies or substantial portions of the Software.
##
## THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
## IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
## FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
## AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
## LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
## OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
## SOFTWARE.

#import math
import sigrokdecode as srd
from .lists import *

class Decoder(srd.Decoder):
    api_version = 3
    id = 'amiccom-rf'
    name = 'Amiccom-RF'
    longname = 'Amiccom RF IC SPI protocl'
    desc = 'Amiccom SPI stack'
    license = 'mit'
    inputs = ['spi']
    outputs = []
    tags = ['SPI', 'IC', 'RF']
    annotations = (
        ('Commands', 'Commands'),
        ('warning', 'Warnings'),
    )
    annotation_rows = (
        ('commands', 'commands', (0,)),
        ('warnings', 'Warnings', (1,)),
    )

    def bitr(self,x):
        return int(bin(x)[2:].zfill(16)[::-1], 2)

    def reset_data(self):
        self.expected = 0
        self.mosi_bytes = []

    def metadata(self, key, value):
       if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def __init__(self):
        self.reset()

    def reset(self):
        self.ss_cmd, self.es_cmd = None, None
        self.reset_data()

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def putx(self, data):
        self.put(self.ss_cmd, self.es_cmd, self.out_ann, data)

    def put_warn(self, pos, msg):
        self.put(pos[0], pos[1], self.out_ann, [2, [msg]])

    def decode(self, ss, es, data):
        ptype = data[0]
        # Don't care about anything else.
        if ptype != 'DATA':
            return
        mosi = data[1]
        #print([data, ss, es])
        #print(self.samplerate)

        if mosi in commands:
            self.put(ss, es, self.out_ann, [0, [commands[mosi]]])

        self.reset()
