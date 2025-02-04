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
    await asyncio.sleep(5)

    print("Moving to a point...")
    await drone.action.goto_location(47.398039859999997, 8.5455725400000002, 10, 0)
    await asyncio.sleep(10)

    print("Landing...")
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(main())
