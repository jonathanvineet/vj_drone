import asyncio
from mavsdk import System

async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    print("Arming the drone...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.takeoff()
    await asyncio.sleep(5)  # Give some time to stabilize

    print("Holding position...")
    await drone.action.hold()  # Command to hover in place

    # Keep the drone in the air indefinitely
    while True:
        await asyncio.sleep(1)  # Prevent CPU overuse

if __name__ == "__main__":
    asyncio.run(main())
