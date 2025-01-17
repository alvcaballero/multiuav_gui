
const apiURL = 'http://localhost:9997/v3/config/paths';

export class cameraModel {

    static async addCameraWebRTC(device) {
        const devicePort = device.ip === '127.0.0.1' ? 8553 : 8554;
        for (let i = 0; i < device.camera.length; i = i + 1) {
            if (device.camera[i]['type'] == 'WebRTC') {
                try {
                    let response = await fetch(
                        `http://localhost:9997/v3/config/paths/add/${device.name}_${device.camera[i].source}`,
                        {
                            method: 'POST',
                            body: JSON.stringify({
                                source: `rtsp://${device.ip}:${devicePort}/${device.camera[i].source}`,
                            }),
                            headers: {
                                'Content-Type': 'application/json',
                            },
                        }
                    );
                    if (response.status == 200) {
                        console.log('camera added ' + device.camera[i].source);
                    } else {
                        console.log(`Error adding camera  ${response.status} ${device.camera[i].source}`);
                        return false;
                    }
                } catch (e) {
                    console.log('\x1b[31m%s\x1b[0m', 'Error adding camera ' + device.camera[i].source);
                    return false;
                }
            }
        }
    }

    static async removeCameraWebRTC(device) {
        if (device.hasOwnProperty('camera')) {
            for (let i = 0; i < device.camera.length; i = i + 1) {
                if (device.camera[i]['type'] == 'WebRTC') {
                    try {
                        await fetch(
                            `${apiURL}/remove/${device.name}_${device.camera[i].source}`,
                            {
                                method: 'POST',
                            }
                        );
                    } catch (e) {
                        console.log('\x1b[31m%s\x1b[0m', 'Error removing camera ' + device.camera[i].source);
                        return false;
                    }
                }
            }
        }
    }
}