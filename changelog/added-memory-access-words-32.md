Added `MemoryInterface::access_words_32`, which performs a mixed list of 32-bit reads and writes in
order and in as few probe transactions as possible. `read_words_32` is now the read-only case of it.
Reading a Cortex-M core register uses it to select the register and fetch the value in one request
rather than three.
