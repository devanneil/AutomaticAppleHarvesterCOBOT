import asyncio
from bleak import BleakClient

ADDRESS = "38:18:2B:71:87:F6"

CHAR_UUID = "abcd1234-5678-90ab-cdef-123456789abc"

async def main():

    async with BleakClient(ADDRESS) as client:

        await client.write_gatt_char(
            CHAR_UUID,
            b"1"
        )

asyncio.run(main())