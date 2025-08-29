//% color=#E3008C weight=100 icon="\uf1d8" block="LN Drone RC"
//% groups=['Radio Control']
namespace droneRC {
    //% block="TestVersion 1"
    //% weight=100
    export function testFunction_TS(): void {
        droneRC.TestFunction_C();
    }

    //% block="initialize SPI and NRF24L01"
    //% weight=90
    export function initNrf24_TS(): boolean {
        return droneRC.InitNrf24_C();
    }

    //% block="throttle=$throttle pitch=$pitch roll=$roll yaw=$yaw"
    //% weight=80
    export function sendRC_TS(throttle: number, pitch: number, roll: number, yaw: number): boolean {
        return droneRC.SendRC_C(throttle, pitch, roll, yaw);
    }
}