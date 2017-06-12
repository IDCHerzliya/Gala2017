'''
keypollertest.py
TAKEN FROM: http://stackoverflow.com/questions/13207678/whats-the-simplest-way-of-detecting-keyboard-input-in-python-from-the-terminal
-------------------------------------------------------
Example implementation of keypoller.py
'''

from keypoller import KeyPoller

with KeyPoller() as keyPoller:
    while True:
        c = keyPoller.poll()
        if not c is None:
            if c == "c":
                break
            print c