// Application Configuration
window.CameraApp = window.CameraApp || {};

window.CameraApp.Config = {
    // API Endpoints
    API: {
        STATUS: '/api/status',
        CONFIG: '/api/config',
        CONFIGURED: '/api/configured',
        NEW_CAMERAS: '/api/new',
        MODELS: '/api/models',
        PREVIEW_ENABLE: '/api/preview/enable',
        PREVIEW: '/api/preview',
        PREVIEW_MJPG: '/api/preview.mjpg',
        ADD_CAMERA: '/api/add',
        DELETE_CAMERA: '/api/delete',
        SETTINGS: '/api/settings',
        SETTINGS_RESET: '/api/settings/reset',
        CAPTURE: '/api/capture',
        ROLES: '/api/roles',
        CALIBRATION: {
            SETUP: '/api/calib/setup',
            STATUS: '/api/calib/status',
            START: '/api/calib/start',
            STOP: '/api/calib/stop',
            MONO: '/api/calib/mono',
            STEREO_CAPTURE: '/api/calib/stereo-capture',
            RUN: '/api/calibration/run',
            PARAMS: '/api/calibration/params',
            START_AUTO: '/api/calibration/start-auto',
            STOP_AUTO: '/api/calibration/stop-auto',
            STATUS_AUTO: '/api/calibration/status-auto',
            COPY_RESULTS: '/api/calibration/copy-results'
        },
        RECORDING: {
            START: '/api/record/start',
            STOP: '/api/record/stop',
            STATUS: '/api/record/status'
        },
        TRACKING: {
            GLOBAL: '/api/tracking/global',
            MODE: '/api/tracking/mode',
            UPDATE: '/api/detections/update'
        }
    },

    // UI Settings
    UI: {
        REFRESH_INTERVAL: 1000,
        GLOBAL_TRACKING_INTERVAL: 2000,
        DETECTION_TIMEOUT: 2000,
        MAX_RETRIES: 3,
        FADE_DURATION: 300
    },

    // Camera Settings
    CAMERA: {
        DEFAULT_WIDTH: 320,
        DEFAULT_HEIGHT: 240,
        MODES: {
            PREVIEW: 'preview',
            DETECT: 'detect',
            CALIBRATION: 'calibration'
        },
        PROFILES: ['auto', 'bright', 'indoor', 'dark'],
        BUFFER_TYPES: ['auto', 'single', 'mplane'],
        ROLES: ['wide_angle', 'zoom', 'zoom_variable']
    },

    // Calibration Settings
    CALIBRATION: {
        DEFAULT_BOARD_COLS: 10,
        DEFAULT_BOARD_ROWS: 7,
        DEFAULT_SQUARE_SIZE: 30.0,
        DEFAULT_DURATION: 30,
        MIN_FRAMES: 10,
        MAX_FRAMES: 100,
        QUALITY_THRESHOLD: 30.0,
        WIDE_ANGLE_THRESHOLD: 80.0
    },

    // Global Tracking Settings
    TRACKING: {
        OBJECT_CLASSES: {
            0: { name: 'person', color: '#e3f2fd', textColor: '#1565c0' },
            1: { name: 'bicycle', color: '#f3e5f5', textColor: '#7b1fa2' },
            2: { name: 'car', color: '#fff3e0', textColor: '#ef6c00' },
            3: { name: 'motorcycle', color: '#e8f5e8', textColor: '#388e3c' },
            7: { name: 'truck', color: '#fff3e0', textColor: '#ef6c00' },
            DEFAULT: { name: 'object', color: '#f5f5f5', textColor: '#616161' }
        }
    },

    // Error Messages
    ERRORS: {
        NETWORK: 'Network error. Please check your connection.',
        TIMEOUT: 'Request timed out. Please try again.',
        SERVER: 'Server error. Please try again later.',
        INVALID_DATA: 'Invalid data received from server.',
        CAMERA_NOT_FOUND: 'Camera not found or not accessible.',
        CALIBRATION_FAILED: 'Calibration process failed.',
        PERMISSION_DENIED: 'Permission denied. Please check your access rights.'
    },

    // Success Messages
    SUCCESS: {
        CAMERA_ADDED: 'Camera added successfully',
        CAMERA_DELETED: 'Camera deleted successfully',
        SETTINGS_SAVED: 'Settings saved successfully',
        CALIBRATION_STARTED: 'Calibration started',
        CALIBRATION_COMPLETED: 'Calibration completed successfully',
        RECORDING_STARTED: 'Recording started',
        RECORDING_STOPPED: 'Recording stopped'
    }
};

// Global application state
window.CameraApp.State = {
    currentTab: 'main',
    previewEnabled: false,
    globalTrackingEnabled: false,
    selectedObjectId: -1,
    lastGlobalObjects: [],
    mainCameras: [],
    configuredCameras: [],
    newCameras: [],
    cameraRoles: [],
    isCalibrating: false,
    isRecording: false,
    advancedSettingsVisible: false,
    currentAdvancedCameraId: null,
    lastGlobalFetch: 0
};

// Utility functions
window.CameraApp.Utils = {
    // Debounce function
    debounce: function(func, wait) {
        let timeout;
        return function executedFunction(...args) {
            const later = () => {
                clearTimeout(timeout);
                func(...args);
            };
            clearTimeout(timeout);
            timeout = setTimeout(later, wait);
        };
    },

    // Format bytes
    formatBytes: function(bytes, decimals = 2) {
        if (bytes === 0) return '0 Bytes';
        const k = 1024;
        const dm = decimals < 0 ? 0 : decimals;
        const sizes = ['Bytes', 'KB', 'MB', 'GB'];
        const i = Math.floor(Math.log(bytes) / Math.log(k));
        return parseFloat((bytes / Math.pow(k, i)).toFixed(dm)) + ' ' + sizes[i];
    },

    // Format duration
    formatDuration: function(seconds) {
        const mins = Math.floor(seconds / 60);
        const secs = seconds % 60;
        return `${mins}:${secs.toString().padStart(2, '0')}`;
    },

    // Generate unique ID
    generateId: function() {
        return Date.now().toString(36) + Math.random().toString(36).substr(2);
    },

    // Safe JSON parse
    safeJsonParse: function(str, defaultValue = null) {
        try {
            return JSON.parse(str);
        } catch (e) {
            return defaultValue;
        }
    },

    // Get object class info
    getObjectClassInfo: function(classId) {
        const config = window.CameraApp.Config.TRACKING.OBJECT_CLASSES;
        return config[classId] || config.DEFAULT;
    },

    // Validate camera ID
    validateCameraId: function(id) {
        return id && typeof id === 'string' && id.length > 0 && !/[^a-zA-Z0-9_-]/.test(id);
    },

    // Create loading element
    createLoadingElement: function(text = 'Loading...') {
        return `
            <div class="d-flex align-items-center justify-content-center p-3">
                <div class="loading-spinner me-2"></div>
                <span>${text}</span>
            </div>
        `;
    },

    // Create error element
    createErrorElement: function(message, showRetry = false) {
        return `
            <div class="alert alert-danger d-flex align-items-center" role="alert">
                <i class="bi bi-exclamation-triangle me-2"></i>
                <div class="flex-grow-1">${message}</div>
                ${showRetry ? '<button class="btn btn-outline-danger btn-sm ms-2" onclick="location.reload()">Retry</button>' : ''}
            </div>
        `;
    }
};