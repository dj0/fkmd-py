import argparse
import serial
from serial.serialposix import Serial
import sys
import os

# print('port=' + str(args.port))

CMD_ADDR = 0
CMD_LEN = 1
CMD_RD = 2
CMD_WR = 3
CMD_RY = 4
CMD_DELAY = 5
PAR_INC = 128
PAR_SINGE = 64
PAR_DEV_ID = 32
PAR_MODE8 = 16


def setAddr(_ser: Serial, addr):
    buff = []
    addr //= 2
    buff.append(CMD_ADDR)
    buff.append((addr >> 16) & 0xFF)
    buff.append(CMD_ADDR)
    buff.append((addr >> 8) & 0xFF)
    buff.append(CMD_ADDR)
    buff.append(addr & 0xFF)
    _ser.write(bytes(buff))


def read(_ser: Serial, buff: bytearray, offset: int, length: int):
    while length > 0:
        rd_len = 65536 if length > 65536 else length
        cmd = []
        cmd.append(CMD_LEN)
        cmd.append((rd_len // 2 >> 8) & 0xFF)
        cmd.append(CMD_LEN)
        cmd.append((rd_len // 2) & 0xFF)
        cmd.append(CMD_RD | PAR_INC)
        _ser.write(bytes(cmd))

        i = 0
        while i < rd_len:
            b = _ser.read(size=rd_len - i)
            buff[offset + i : (offset + i) + (rd_len - i)] = b
            i += len(b)
        length -= rd_len
        offset += rd_len


def getID(_ser: Serial):
    cmd = [CMD_RD | PAR_SINGE | PAR_DEV_ID]
    # print(bytes(cmd).hex()) debug
    _ser.write(bytes(cmd))
    id = int.from_bytes(_ser.read(), "big") << 8
    id |= int.from_bytes(_ser.read(), "big")
    # print(id) debug
    return id


def setDelay(_ser: Serial, val):
    cmd = [CMD_DELAY, val]
    _ser.write(bytes(cmd))


def getRomRegion(rom_hdr: bytearray):
    val = rom_hdr[0x1F0]
    if val != rom_hdr[0x1F1] and rom_hdr[0x1F1] != 0x20 and rom_hdr[0x1F1] != 0:
        return "W"
    if val in {ord("F"), ord("C")}:
        return "W"
    elif val in {ord("U"), ord("W"), ord("4"), 4}:
        return "U"
    elif val in {ord("J"), ord("B"), ord("1"), 1}:
        return "J"
    elif val in {ord("E"), ord("A"), ord("8"), 8}:
        return "E"
    return "X"


def getRomName(_ser: Serial):
    setAddr(_ser, 0)
    rom_hdr = bytearray([0] * 512)
    read(_ser, rom_hdr, 0, 512)
    name = romNameParse(0x120, rom_hdr)
    if name is None:
        name = romNameParse(0x150, rom_hdr)
    if name is None:
        name = "Unknown"
    name += f" ({getRomRegion(rom_hdr)})"
    return name


def romNameParse(offset, buff: bytearray):
    name = ""
    name_empty = 1

    i = offset + 47
    while i >= offset:
        if buff[i] != 0 and buff[i] != 0x20:
            break
        if buff[i] == 0x20:
            buff[i : i + 1] = bytearray([0x00])
        i -= 1

    i = offset
    while i < offset + 48:
        if buff[i] == 0:
            break
        if buff[i] == ord("/") | buff[i] == ord(":"):
            buff[i : i + 1] = bytearray([ord("-")])
        try:
            name += buff[i : i + 1].decode(encoding="ascii")
            if buff[i] != 0x20:
                name_empty = 0
        except:
            return None
        i += 1
    if name_empty != 0:
        return None
    return name


def ramAvailable(_ser: Serial):
    writeWord(_ser, 0xA13000, 0xFFFF)
    first_word = readWord(_ser, 0x200000)
    writeWord(_ser, 0x200000, first_word ^ 0xFFFF)
    tmp = readWord(_ser, 0x200000)
    writeWord(_ser, 0x200000, first_word)
    tmp ^= 0xFFFF
    if (first_word & 0x00FF) != (tmp & 0x00FF):
        return False
    return True


def writeWord(_ser: Serial, addr, data):
    addr //= 2
    cmd = []
    cmd.append(CMD_ADDR)
    cmd.append(addr >> 16 & 0xFF)
    cmd.append(CMD_ADDR)
    cmd.append(addr >> 8 & 0xFF)
    cmd.append(CMD_ADDR)
    cmd.append(addr & 0xFF)

    cmd.append(CMD_WR | PAR_SINGE)
    cmd.append(data >> 8 & 0xFF)
    cmd.append(data & 0xFF)

    _ser.write(bytes(cmd))


def writeByte(_ser: Serial, addr, data):
    addr //= 2
    cmd = []
    cmd.append(CMD_ADDR)
    cmd.append(addr >> 16 & 0xFF)
    cmd.append(CMD_ADDR)
    cmd.append(addr >> 8 & 0xFF)
    cmd.append(CMD_ADDR)
    cmd.append(addr & 0xFF)

    cmd.append(CMD_WR | PAR_SINGE | PAR_MODE8)
    cmd.append(data & 0xFF)

    _ser.write(bytes(cmd))


def readWord(_ser: Serial, addr):
    val = 0
    addr //= 2

    cmd = []
    cmd.append(CMD_ADDR)
    cmd.append(addr >> 16 & 0xFF)
    cmd.append(CMD_ADDR)
    cmd.append(addr >> 8 & 0xFF)
    cmd.append(CMD_ADDR)
    cmd.append(addr & 0xFF)
    cmd.append(CMD_RD | PAR_SINGE)
    _ser.write(bytes(cmd))
    val = int.from_bytes(_ser.read(), "big")
    val = val | int.from_bytes(_ser.read(), "big")
    return val


def checkRomSize(_ser: Serial, base_addr, max_len):
    base_len = 0x8000
    length = 0x8000
    sector0 = bytearray([0] * 256)
    sector = bytearray([0] * 256)
    writeWord(_ser, 0xA13000, 0x0000)
    setAddr(_ser, base_addr)
    read(_ser, sector0, 0, len(sector))

    while True:
        setAddr(_ser, base_addr + length)
        read(_ser, sector, 0, len(sector))

        eq = 1
        i = 0
        while i < len(sector):
            if sector0[i] != sector[i]:
                eq = 0
            i += 1
        if eq == 1:
            break

        length *= 2
        if length >= max_len:
            break

    if length == base_len:
        return 0
    return length


def getRomSize(_ser: Serial):
    sector0 = bytearray([0] * 512)
    sector = bytearray([0] * 512)
    ram = 0
    extra_rom = 0

    if ramAvailable(_ser):
        ram = 1
        extra_rom = 1
        writeWord(0xA13000, 0x0000)
        setAddr(_ser, 0x200000)
        read(_ser, sector0, 0, 512)
        setAddr(_ser, 0x200000)
        read(_ser, sector, 0, 512)

        i = 0
        while i < len(sector):
            if sector[i] != sector0[i]:
                extra_rom = 0
            i += 1

        if extra_rom != 0:
            extra_rom = 0
            setAddr(_ser, 0x200000 + 0x10000)
            read(_ser, sector, 0, 512)

            writeWord(_ser, 0xA13000, 0xFFFF)
            setAddr(_ser, 0x200000)
            read(_ser, sector, 0, 512)
            i = 0
            while i < len(sector):
                if sector[i] != sector0[i]:
                    extra_rom = 1
                i += 1

    max_rom_size = 0x200000 if ram != 0 and extra_rom == 0 else 0x400000
    length = checkRomSize(_ser, 0, max_rom_size)

    if length == 0x400000:
        length = 0x200000
        length2 = checkRomSize(_ser, 0x200000, 0x200000)
        if length2 == 0x200000:
            length2 = checkRomSize(_ser, 0x300000, 0x100000)
            length2 = 0x200000 if length2 >= 0x80000 else 0x100000
        if length2 >= 0x80000:
            length += length2

    return length


def readRom(_ser: Serial):
    try:
        block_size = 32768
        rom_name = getRomName(_ser)
        rom_name += ".bin"
        rom_size = getRomSize(_ser)
        rom = bytearray()

        print(f"Read ROM to {rom_name}")
        print(f"ROM Size: {rom_size // 1024}K")

        writeWord(_ser, 0xA13000, 0x0000)
        setAddr(_ser, 0)
        i = 0
        while i < rom_size:
            read(_ser, rom, i, block_size)
            i += block_size
        with open(rom_name, mode="wb") as f:
            f.write(rom)
            print("ROM dumped")

    except Exception as e:
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        print(f"{exc_type}, {fname}, {exc_tb.tb_lineno}")
        print(e)
        print("ROM dumping failed")


def flashUnlockByPass(_ser: Serial):
    writeWord(_ser, 0x555 * 2, 0xAA)
    writeByte(_ser, 0x2AA * 2, 0x55)
    writeByte(_ser, 0x555 * 2, 0x20)


def flashResetByPass(_ser: Serial):
    writeWord(_ser, 0, 0xF0)
    writeByte(_ser, 0, 0x90)
    writeByte(_ser, 0, 0x00)


def flashRY(_ser: Serial):
    cmd = []
    cmd.append(CMD_RY)
    cmd.append(CMD_RD | PAR_SINGE)
    _ser.write(bytes(cmd))
    _ser.read()
    _ser.read()


def flashErase(_ser: Serial, addr):
    cmd = bytearray([0] * 8 * 8)
    addr //= 2

    i = 0
    while i < len(cmd):
        cmd[0 + i] = CMD_ADDR
        cmd[1 + i] = addr >> 16 & 0xFF
        cmd[2 + i] = CMD_ADDR
        cmd[3 + i] = addr >> 8 & 0xFF
        cmd[4 + i] = CMD_ADDR
        cmd[5 + i] = addr & 0xFF
        cmd[6 + i] = CMD_WR | PAR_SINGE | PAR_MODE8
        cmd[7 + i] = 0x30
        addr += 4096
        i += 8

    writeWord(_ser, 0x555 * 2, 0xAA)
    writeWord(_ser, 0x2AA * 2, 0x55)
    writeWord(_ser, 0x555 * 2, 0x80)
    writeWord(_ser, 0x555 * 2, 0xAA)
    writeWord(_ser, 0x2AA * 2, 0x55)
    _ser.write(cmd)
    flashRY(_ser)


def flashWrite(_ser: Serial, buff: bytearray, offset, length):
    length //= 2
    cmd = bytearray([0] * 6 * length)
    i = 0
    while i < len(cmd):
        cmd[0 + i] = CMD_WR | PAR_SINGE | PAR_MODE8
        cmd[1 + i] = 0xA0
        cmd[2 + i] = CMD_WR | PAR_SINGE | PAR_INC
        cmd[3 + i] = buff[offset]
        offset += 1
        cmd[4 + i] = buff[offset]
        offset += 1
        cmd[5 + i] = CMD_RY
        i += 6
    _ser.write(cmd)


def writeRom(_ser: Serial, romfile):
    try:
        block_len = 4096
        with open(romfile, mode="rb") as f:
            rom = f.read()
            rom_size = len(rom)
            print("Flash erase...")
            flashResetByPass(_ser)

            i = 0
            while i < rom_size:
                flashErase(_ser, i)
                i += 65536

            print("Flash write...")
            flashUnlockByPass(_ser)
            setAddr(_ser, 0)
            i = 0
            while i < rom_size:
                flashWrite(_ser, rom, i, block_len)
                i += block_len
            flashResetByPass(_ser)

            print("Flash verify...")
            rom2 = bytearray([0] * len(rom))
            setAddr(_ser, 0)
            i = 0
            while i < rom_size:
                read(_ser, rom2, i, block_len)
                i += block_len

            i = 0
            while i < rom_size:
                if rom[i] != rom2[i]:
                    raise Exception(f"Verify error at {i}")
                i += 1
            print("OK")

    except Exception as e:
        exc_type, exc_obj, exc_tb = sys.exc_info()
        fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
        print(f"{exc_type}, {fname}, {exc_tb.tb_lineno}")
        print(e)
        print("failed to write ROM")
        try:
            flashResetByPass(_ser)
        except Exception as e:
            print(e)


# ---program start
parser = argparse.ArgumentParser()

parser.add_argument("-port", required=True, type=str, help="serial port to use")
parser.add_argument("-readrom", action="store_true", help="Read and save ROM")
parser.add_argument("-writerom", type=str, help="Write ROM")
# parser.add_argument("-autoname", action='store_true', help='Read ROM name and generate filenames to save ROM/RAM data')
# parser.add_argument("-rominfo", action='store_true', help='Print ROM info')

args = parser.parse_args()


with serial.Serial(args.port, 9600, timeout=0.2, write_timeout=0.2) as ser:
    ser.flush()
    id = getID(ser)
    if (id & 0xFF) == (id >> 8) and id != 0:
        setDelay(ser, 0)
        ser.write_timeout = 2
        ser.timeout = 2

    if args.readrom:
        setDelay(ser, 1)
        readRom(ser)

    if args.writerom:
        setDelay(ser, 0)
        writeRom(ser, args.writerom)
