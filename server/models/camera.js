
const apiURL = 'http://localhost:9997/v3/config/paths';

export class cameraModel {

    static async addCameraWebRTC(device) {
        for (let i = 0; i < device.camera.length; i = i + 1) {
            if (device.camera[i]['type'] == 'WebRTC') {
                await fetch(`${apiURL}/add/${device.name}_${device.camera[i].source}`, {
                    method: 'POST',
                    body: JSON.stringify({
                        source: `rtsp://${device.ip}:8554/${device.camera[i].source}`,
                    }),
                    headers: {
                        'Content-Type': 'application/json',
                    },
                });
            }
        }
    }


    static async removeCameraWebRTC(device) {
        if (device.hasOwnProperty('camera')) {
            for (let i = 0; i < device.camera.length; i = i + 1) {
                if (device.camera[i]['type'] == 'WebRTC') {
                    await fetch(
                        `${apiURL}/remove/${device.name}_${device.camera[i].source}`,
                        {
                            method: 'POST',
                        }
                    );
                }
            }
        }
    }
}