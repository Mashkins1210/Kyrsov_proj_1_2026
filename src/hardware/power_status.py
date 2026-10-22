"""Decode Raspberry Pi current and historical throttling separately."""
import subprocess

def decode_status(raw):
    if not isinstance(raw,str) or not raw.strip().startswith('throttled=0x'):
        raise ValueError('Invalid Raspberry Pi power status')
    value=int(raw.strip().split('=',1)[1],16)
    return {'raw':raw.strip(),'current_fault':bool(value&15),
            'undervoltage_now':bool(value&1),'throttled_now':bool(value&4),
            'undervoltage_occurred':bool(value&(1<<16)),
            'throttling_occurred':bool(value&(1<<18))}

def read_status():
    return decode_status(subprocess.check_output(['vcgencmd','get_throttled'],text=True,timeout=1))

