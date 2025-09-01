//% color=#EC1313 weight=91 icon="\uf19c" block="LU-Bit"
namespace luBit {
    //% block="TestVersion 1"
    //% weight=100
    //% group="Test"
    export function testFunction_TS(): void {
        luBit.TestFunction_C();
    }

    //% block="initialize SPI and NRF24L01"
    //% weight=90
    //% group="LU_Drone"
    export function initNrf24_TS(): boolean {
        return luBit.InitNrf24_C();
    }

    //% block="throttle=$throttle pitch=$pitch roll=$roll yaw=$yaw"
    //% weight=80
    //% group="LU_Drone"
    export function sendRC_TS(throttle: number, pitch: number, roll: number, yaw: number): boolean {
        return luBit.SendRC_C(throttle, pitch, roll, yaw);
    }
}
