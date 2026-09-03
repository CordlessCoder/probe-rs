A CMSIS-DAP probe interrupted mid-transfer no longer has to be physically reconnected. It answers
every later command with the previous one's reply, which survives closing and reopening the device;
probe-rs now prompts the outstanding replies out of it at open and checks that requests and replies
are back in step before trusting what it is told.
