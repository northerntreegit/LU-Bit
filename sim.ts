namespace pxsim {
    /**
     * Simulator support for droneRadio functions
     */
    export namespace droneRadio {
        export function testFunction_TS(): void {
            console.log("testFunction");
            // In simulator, just log the action
        }

        export function initNrf24_TS(): boolean {
            console.log("initNrf24");
            return true;
            // In simulator, just log the action
        }
        
        export function sendRC_TS(throttle: number, pitch: number, roll: number, yaw: number): boolean {
            console.log(`Simulator: SendRC called with throttle=${throttle}, pitch=${pitch}, roll=${roll}, yaw=${yaw}`);
            // In simulator, just log the action
            return true;
        }
    }
}
