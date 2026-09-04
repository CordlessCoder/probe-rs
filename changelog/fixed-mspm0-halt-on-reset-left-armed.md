MSPM0: detaching now restores `DPREC0.RST CTL` to its default. Halt-on-reset was left selected
after a session ended, so any later reset stopped the core with no debugger present to release it.
