// API Communication Layer
window.CameraApp = window.CameraApp || {};

window.CameraApp.API = {
    // Base fetch wrapper with error handling
    async request(url, options = {}) {
        const config = {
            method: 'GET',
            headers: {
                'Content-Type': 'application/json',
                ...options.headers
            },
            cache: 'no-cache',
            ...options
        };

        try {
            const response = await fetch(url, config);
            
            if (!response.ok) {
                throw new Error(`HTTP ${response.status}: ${response.statusText}`);
            }

            const contentType = response.headers.get('content-type');
            if (contentType && contentType.includes('application/json')) {
                return await response.json();
            } else {
                return await response.text();
            }
        } catch (error) {
            console.error(`API Error for ${url}:`, error);
            throw error;
        }
    },

    // Camera specific API methods
    //getCameraHealth(detPort) {
    //    return this.request(`${location.protocol}//${location.hostname}:${detPort}/api/health`);
    //},
    async getCameraHealth(detPort) {
    try {
        const response = await fetch(`${location.protocol}//${location.hostname}:${detPort}/api/health`, {
            cache: 'no-store',
            mode: 'cors',
            credentials: 'omit'
        });
        return await response.json();
        } catch (error) {
        console.warn(`Health check failed for port ${detPort}:`, error.message);
        return { cap_real_fps: 0 }; // Возвращаем дефолтное значение
        }
    },
    getPreviewMjpgUrl(cameraId) {
        return `/api/preview.mjpg?id=${encodeURIComponent(cameraId)}`;
    },

    getCameraStreamUrl(detPort) {
        return `${location.protocol}//${location.hostname}:${detPort}/api/stream.mjpg`;
    },

    getNewCameraPreviewUrl(byIdPath) {
        return `/api/preview?by=${encodeURIComponent(byIdPath)}&t=${Date.now()}`;
    },

    // Status and Config APIs
    async getStatus() {
        return await this.request(window.CameraApp.Config.API.STATUS);
    },

    async getConfig() {
        return await this.request(window.CameraApp.Config.API.CONFIG);
    },

    async enablePreview(enabled) {
        return await this.request(window.CameraApp.Config.API.PREVIEW_ENABLE, {
            method: 'POST',
            body: JSON.stringify({ enable: enabled })
        });
    },

    // Camera Management APIs
    async getConfiguredCameras() {
        return await this.request(window.CameraApp.Config.API.CONFIGURED);
    },

    async getNewCameras() {
        return await this.request(window.CameraApp.Config.API.NEW_CAMERAS);
    },

    async getModels() {
        return await this.request(window.CameraApp.Config.API.MODELS);
    },

    async addCamera(id, byIdPath) {
        return await this.request(window.CameraApp.Config.API.ADD_CAMERA, {
            method: 'POST',
            body: JSON.stringify({ id, by_id: byIdPath })
        });
    },

    async deleteCamera(id) {
        return await this.request(window.CameraApp.Config.API.DELETE_CAMERA, {
            method: 'POST',
            body: JSON.stringify({ id })
        });
    },

    async setCameraMode(id, mode) {
        return await this.request(window.CameraApp.Config.API.PREVIEW, {
            method: 'POST',
            body: JSON.stringify({ id, mode })
        });
    },

    async getCameraRoles() {
        return await this.request(window.CameraApp.Config.API.ROLES);
    },

    async setCameraRole(id, role) {
        return await this.request(window.CameraApp.Config.API.ROLES, {
            method: 'POST',
            body: JSON.stringify({ id, role })
        });
    },


    async updateCameraSettings(id, settings) {
        return await this.request(window.CameraApp.Config.API.SETTINGS, {
            method: 'POST',
            body: JSON.stringify({ id, ...settings })
        });
    },

    async resetCameraSettings(id) {
        return await this.request(window.CameraApp.Config.API.SETTINGS_RESET, {
            method: 'POST',
            body: JSON.stringify({ id })
        });
    },

    // Capture APIs
    async captureFrame(id, timestamp = null) {
        const url = timestamp 
            ? `${window.CameraApp.Config.API.CAPTURE}?id=${encodeURIComponent(id)}&t=${timestamp}`
            : `${window.CameraApp.Config.API.CAPTURE}?id=${encodeURIComponent(id)}`;
        return await this.request(url);
    },

    // Calibration APIs
    async setupCalibration(camera) {
        return await this.request(window.CameraApp.Config.API.CALIBRATION.SETUP, {
            method: 'POST',
            body: JSON.stringify({ camera })
        });
    },

    async getCalibrationStatus(camera = null) {
        const url = camera 
            ? `${window.CameraApp.Config.API.CALIBRATION.STATUS}?camera=${encodeURIComponent(camera)}`
            : window.CameraApp.Config.API.CALIBRATION.STATUS;
        return await this.request(url);
    },

    async startCalibration() {
        return await this.request(window.CameraApp.Config.API.CALIBRATION.START, {
            method: 'POST'
        });
    },

    async stopCalibration() {
        return await this.request(window.CameraApp.Config.API.CALIBRATION.STOP, {
            method: 'POST'
        });
    },

    async runCalibration(params) {
        return await this.request(window.CameraApp.Config.API.CALIBRATION.RUN, {
            method: 'POST',
            body: JSON.stringify(params)
        });
    },

    async getCalibrationParams() {
        return await this.request(window.CameraApp.Config.API.CALIBRATION.PARAMS);
    },

    async setCalibrationParams(params) {
        return await this.request(window.CameraApp.Config.API.CALIBRATION.PARAMS, {
            method: 'POST',
            body: JSON.stringify(params)
        });
    },

    async startAutoCalibration() {
        return await this.request(window.CameraApp.Config.API.CALIBRATION.START_AUTO, {
            method: 'POST'
        });
    },

    async stopAutoCalibration() {
        return await this.request(window.CameraApp.Config.API.CALIBRATION.STOP_AUTO, {
            method: 'POST'
        });
    },

    async getAutoCalibrationStatus() {
        return await this.request(window.CameraApp.Config.API.CALIBRATION.STATUS_AUTO);
    },

    async copyCalibrationResults() {
        return await this.request(window.CameraApp.Config.API.CALIBRATION.COPY_RESULTS, {
            method: 'POST'
        });
    },

    // Recording APIs
    async startRecording(duration = 30) {
        return await this.request(window.CameraApp.Config.API.RECORDING.START, {
            method: 'POST',
            body: JSON.stringify({ duration })
        });
    },

    async stopRecording() {
        return await this.request(window.CameraApp.Config.API.RECORDING.STOP, {
            method: 'POST'
        });
    },

    // Global Tracking APIs
    async setTrackingMode(global = false) {
        return await this.request(window.CameraApp.Config.API.TRACKING.MODE, {
            method: 'POST',
            body: JSON.stringify({ global })
        });
    },

    async getGlobalTracking() {
        return await this.request(window.CameraApp.Config.API.TRACKING.GLOBAL);
    },

    async updateDetections() {
        return await this.request(window.CameraApp.Config.API.TRACKING.UPDATE);
    },

    // Camera-specific APIs (for individual camera servers)
    async highlightObject(cameraPort, trackId) {
        return await this.request(`${location.protocol}//${location.hostname}:${cameraPort}/api/highlight`, {
            method: 'POST',
            body: JSON.stringify({ track_id: trackId })
        });
    },

    async getCameraDetections(cameraPort) {
        return await this.request(`${location.protocol}//${location.hostname}:${cameraPort}/api/last.json`);
    },

    async getCameraHealth(cameraPort) {
        return await this.request(`${location.protocol}//${location.hostname}:${cameraPort}/api/health`);
    },

    // Utility functions
    getPreviewUrl(id) {
        return `${window.CameraApp.Config.API.PREVIEW}?id=${encodeURIComponent(id)}`;
    },

    getPreviewMjpgUrl(id) {
        return `${window.CameraApp.Config.API.PREVIEW_MJPG}?id=${encodeURIComponent(id)}`;
    },

    getNewCameraPreviewUrl(byId) {
        return `${window.CameraApp.Config.API.PREVIEW}?by=${encodeURIComponent(byId)}&t=${Date.now()}`;
    },

    getCameraStreamUrl(port) {
        return `${location.protocol}//${location.hostname}:${port}/api/stream.mjpg`;
    }
};