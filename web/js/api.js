// API Communication Layer
window.CameraApp = window.CameraApp || {};

window.CameraApp.API = {
    _healthCache: new Map(),
    _healthThrottleMs: 300,
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
        const now = (typeof performance !== 'undefined' && performance.now)
            ? performance.now()
            : Date.now();
        const cacheEntry = this._healthCache.get(detPort);
        if (cacheEntry) {
            if (cacheEntry.promise) {
                return cacheEntry.promise;
            }
            if (cacheEntry.data && now - cacheEntry.timestamp < this._healthThrottleMs) {
                return cacheEntry.data;
            }
        }

        const fetchPromise = (async () => {
            try {
                const response = await fetch(`${location.protocol}//${location.hostname}:${detPort}/api/health`, {
                    cache: 'no-store',
                    mode: 'cors',
                    credentials: 'omit'
                });
                const data = await response.json();
                this._healthCache.set(detPort, {
                    timestamp: (typeof performance !== 'undefined' && performance.now)
                        ? performance.now()
                        : Date.now(),
                    data
                });
                return data;
            } catch (error) {
                console.warn(`Health check failed for port ${detPort}:`, error.message);
                const fallback = { cap_real_fps: 0 };
                this._healthCache.set(detPort, {
                    timestamp: (typeof performance !== 'undefined' && performance.now)
                        ? performance.now()
                        : Date.now(),
                    data: fallback
                });
                return fallback;
            }
        })();

        this._healthCache.set(detPort, {
            timestamp: now,
            promise: fetchPromise,
            data: cacheEntry && cacheEntry.data ? cacheEntry.data : null
        });

        return fetchPromise;
    },
    getPreviewMjpgUrl(cameraId) {
        return `/api/preview.mjpg?id=${encodeURIComponent(cameraId)}`;
    },


    // Status and Config APIs
    async getStatus() {
        return await this.request(window.CameraApp.Config.API.STATUS);
    },

    async getConfig() {
        return await this.request(window.CameraApp.Config.API.CONFIG);
    },


    async setManagerDebugLogging(enabled) {
        return await this.request(window.CameraApp.Config.API.CONFIG, {
            method: 'POST',
            body: JSON.stringify({ manager_debug_enabled: enabled })
        });
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

    async addCamera(id, match) {
        const payload = { id };
        if (typeof match === 'string') {
            payload.match = { type: 'by-id', value: match };
        } else if (match && typeof match === 'object') {
            payload.match = {
                type: match.type || 'by-id',
                value: match.value || ''
            };
            if (match.device) {
                payload.match.device = match.device;
            }
        }
        return await this.request(window.CameraApp.Config.API.ADD_CAMERA, {
            method: 'POST',
            body: JSON.stringify(payload)
        });
    },

    async reassignCamera(id, match) {
        const payload = { id };
        if (match && typeof match === 'object') {
            payload.match = {
                type: match.type || 'by-id',
                value: match.value || ''
            };
            if (match.device) {
                payload.match.device = match.device;
            }
        }
        return await this.request(window.CameraApp.Config.API.REASSIGN_CAMERA, {
            method: 'POST',
            body: JSON.stringify(payload)
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

    // Live calibration (new API)
    async getLiveCalibrationCameras() {
        return await this.request(window.CameraApp.Config.API.CALIBRATION_NEW.CAMERAS);
    },

    async startLiveCalibration(params = {}) {
        return await this.request(window.CameraApp.Config.API.CALIBRATION_NEW.START, {
            method: 'POST',
            body: JSON.stringify(params)
        });
    },

    async stopLiveCalibration() {
        return await this.request(window.CameraApp.Config.API.CALIBRATION_NEW.STOP, {
            method: 'POST'
        });
    },

    async calibrateLiveCalibration() {
        return await this.request(window.CameraApp.Config.API.CALIBRATION_NEW.CALIBRATE, {
            method: 'POST'
        });
    },

    async computeLiveCalibration() {
        return await this.request(window.CameraApp.Config.API.CALIBRATION_NEW.COMPUTE, {
            method: 'POST'
        });
    },

    async getLiveCalibrationStatus() {
        return await this.request(window.CameraApp.Config.API.CALIBRATION_NEW.STATUS);
    },


    // Global Tracking APIs
    async setTrackingMode(global = false) {
        return await this.request(window.CameraApp.Config.API.TRACKING.MODE, {
            method: 'POST',
            body: JSON.stringify({ global })
        });
    },

    async setGrayscaleTrackingMode(grayscale = false) {
        return await this.request(window.CameraApp.Config.API.TRACKING.GRAYSCALE_MODE, {
            method: 'POST',
            body: JSON.stringify({ grayscale })
        });
    },

    async getGrayscaleTrackingMode() {
        return await this.request(window.CameraApp.Config.API.TRACKING.GRAYSCALE_MODE);
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

    getNewCameraPreviewUrl(identifier) {
        const timestamp = `&t=${Date.now()}`;
        if (!identifier) {
            return `${window.CameraApp.Config.API.PREVIEW}?${timestamp.slice(1)}`;
        }

        const buildUrl = (param, value) =>
            `${window.CameraApp.Config.API.PREVIEW}?${param}=${encodeURIComponent(value)}${timestamp}`;

        if (typeof identifier === 'string') {
            return buildUrl('by', identifier);
        }

        const { type, value } = identifier;
        if (!value) {
            return `${window.CameraApp.Config.API.PREVIEW}?${timestamp.slice(1)}`;
        }

        if (type === 'by-path') {
            return buildUrl('path', value);
        }
        if (type === 'device') {
            return buildUrl('dev', value);
        }
        return buildUrl('by', value);
    },

    getCameraStreamUrl(port) {
        return `${location.protocol}//${location.hostname}:${port}/api/stream.mjpg`;
    }
};